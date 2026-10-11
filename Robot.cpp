#include "Robot.h"


bool Robot::alreadyConfigurated = false;
double Robot::maxVelocity = 0, Robot::minDistance;
double Robot::waypointDist;
string Robot::folder;
string Robot::log_name;
bool Robot::isHolonomic;
bool Robot::savePosVel, Robot::saveArrivedLeftRobots;
unsigned long long Robot::testTime;
double Robot::d;
double Robot::D;
double Robot::maxLinSpeed;
double Robot::instantTotalEnergy;
vector<bool> *Robot::energyWasSummed;
ofstream Robot::logE, Robot::arrivedLeftLog;
bool Robot::saveEnergy;
double Robot::LASER_FOV=-1, Robot::LASER_SAMPLES=-1;

/* Constructor arguments:
    confFile: pointer to the ConfFile class with experiments variables.*/
Robot::Robot(ConfigFile *confFile)
{
  readConfigFile(confFile);
}

//Initialize all robot data (pose, log file, velocity, etc.)
void Robot::init(int id)
{
  m_id = id;
  m_name = "robot" + intToStr(id);
  Pose pose = pos->GetPose();
  m_x  = pose.x;
  m_y = pose.y;
  m_th = pose.a;
  numIterations = numIterationsReachGoal = 0;
  FinalLog::init(folder+"/"+log_name);
  finished = finishedBySimTime = false;
  stalls = 0; 
  alreadyStalled = false;
  exitedFromTargetRegion = false;
  alreadyChanged = 1;
  destinationX = waypoints[0][0];
  destinationY = waypoints[0][1];
  fx = fy = 0;
  currentWaypoint = 0;
  linSpeed = rotSpeed = 0;
  mean_distance = d; var_distance = 0; mean_velocity = 0; var_velocity = 0;
  n_distances = n_velocities = 0;
  if (savePosVel){
    posVelLog.open((folder+"/"+m_name).c_str());
    writeInPosVelLog(0,0);
  }
  if (saveEnergy){
    U0 = Ka*hypot(m_x - destinationX, m_y - destinationY);
  }
  init_laser();
}

//Finishes a robot. Only displays a message for looking with the simulation is still running.
void Robot::finish()
{
  //This message is to see how many robots ended while experimentation scripts are ongoing, then I can see if it is stopped.
  cout << "Robot " << m_id << " finished!" << endl;
  if (savePosVel){
    posVelLog.close();
  }
}

// write in posVelLog the robot's velocity, position and acceleration. 
void Robot::writeInPosVelLog(double x_acc, double y_acc)
{
  posVelLog
    << theWorld->SimTimeNow() << " "
    << setprecision(17)
    << m_x << " "
    << m_y << " "
    << linSpeed << " "
    << rotSpeed << " "
    << x_acc << " "
    << y_acc << " "
    << endl;
}

// write in posVelLog the robot's output velocity, position and acceleration. 
void Robot::writeInPosOutVelLog(double linvec)
{
  posVelLog
    << theWorld->SimTimeNow() << " "
    << setprecision(17)
    << m_x << " "
    << m_y << " "
    << linvec
    << " output"
    << endl;
}

// write in arrivedLeftLog when robot left or arrived.
void Robot::writeInArrivedLeftLog(World* theWorld, unsigned int m_id, bool arrived)
{
  arrivedLeftLog
    << theWorld->SimTimeNow() << " "
    << setprecision(17)
    << m_id << " ";
  if (arrived)
    arrivedLeftLog << "arrived" << endl;
  else
    arrivedLeftLog << "left" << endl;
}

//Subtract two angle values (in radians). The result value lies between -2 PI and 2 PI. 
double Robot::angDiff(double end, double begin)
{
   double returnMe = end - begin;
  
   if (returnMe > PI)
      returnMe = -(2*PI - returnMe);
   else if (returnMe < -PI)
      returnMe = 2*PI + returnMe;
    
   return returnMe;
}

//Initializes values for sensing with laser, depending on the world file used.
void Robot::init_laser()
{
  //init laser configuration for use in getBearing()
  ModelRanger::Sensor sensor = laser->GetSensors()[0];
  if (LASER_FOV == -1 && LASER_SAMPLES == -1){
    LASER_FOV  = rtod(sensor.fov);
    LASER_SAMPLES = sensor.sample_count;
  }
  laser->vis.showArea.set(0);
}

/* Calculate x- and y-speed from force vector (fx,fy).
  Outputs linSpeed and rotSpeed.
  Observation:
    linSpeed is repurposed to signify x_speed.
    rotSpeed is repurposed to signify y_speed. */
void Robot::calculateHolonomicSpeed()
{
  /* Control for omni robot */
  double x_accel = fx;
  double y_accel = fy;
  
  x_accel += -Kdp*linSpeed;
  y_accel += -Kdp*rotSpeed;
  
  linSpeed = linSpeed + x_accel*(TIME_STEP);
  rotSpeed = rotSpeed + y_accel*(TIME_STEP);
  
  if (savePosVel) writeInPosVelLog(x_accel, y_accel);
}

/* Calculate x- and y-speed from force vector (fx,fy).
  Changes linSpeed and rotSpeed. */
void Robot::calculateNonHolonomicSpeed()
{
  double angTarget;
  if (fx == 0 && fy == 0) 
      angTarget = m_th;
  else  
      angTarget = atan2(fy, fx);
  
  double linAccel = Kl*(fx*cos(m_th) + fy*sin(m_th));
  double rotAccel = Kr*(angDiff(angTarget, m_th));
  
  linAccel += -Kdp*linSpeed;
  rotAccel += -Kdp*rotSpeed;
  
  linSpeed = linSpeed + linAccel*(TIME_STEP);
  rotSpeed = rotSpeed + rotAccel*(TIME_STEP);
  
  if (savePosVel) writeInPosVelLog(linAccel, rotAccel);
}

/* Updates maximum, mean and variance of velocity for holonomic robots only if the robot does not finished. */
void Robot::updateHolonomicVelocityStatistics()
{
  if (!finished){
    Velocity veloc = pos->GetVelocity();
    double linvec = hypot(veloc.x,veloc.y);
    if (linvec > maxVelocity) 
      maxVelocity = linvec;
    n_velocities++;
    IterativeMeanVariance(linvec, n_velocities, &mean_velocity, &var_velocity);
    if (saveEnergy){
      calculateEnergy(linvec,hypot(linSpeed,rotSpeed));
    }
    //~ if (saveEnergy && !(*energyWasSummed)[m_id])
      //~ sumInstantEnergy(m_id, calculateEnergy(linvec), theWorld);
    if (savePosVel) writeInPosOutVelLog(linvec);
  }
}

/* Updates maximum, mean and variance of velocity for non-holonomic robots only if the robot does not finished.*/
void Robot::updateNonHolonomicVelocityStatistics()
{
  if (!finished){
    Velocity veloc = pos->GetVelocity();
    if (fabs(veloc.x) > maxVelocity) 
      maxVelocity = fabs(veloc.x); 
    n_velocities++;
    IterativeMeanVariance(veloc.x, n_velocities, &mean_velocity, &var_velocity);
    if (saveEnergy)
      calculateEnergy(veloc.x,hypot(linSpeed,rotSpeed));
    //~ if (saveEnergy && !(*energyWasSummed)[m_id])
      //~ sumInstantEnergy(m_id, calculateEnergy(veloc.x), theWorld);
    if (savePosVel) writeInPosOutVelLog(veloc.x);
  }
}

// Set linear and turning speeds based on the force vector. The linear velocity is set to a maximum value.
void Robot::setSpeeds()
{
  if (isHolonomic){
    calculateHolonomicSpeed();

    // suppress maximum velocity
    double normspeed = hypot(linSpeed,rotSpeed);
    if (normspeed > maxLinSpeed){
      linSpeed = maxLinSpeed*linSpeed/normspeed;
      rotSpeed = maxLinSpeed*rotSpeed/normspeed;
    }
    
    if (!finished && saveEnergy){
      logE << setfill(' ') << setw(9);
      logE << theWorld->SimTimeNow();
      logE << ", " << m_id << ":";
      logE.precision(6);
      logE.setf(ios::fixed);
      logE << " vi=" << normspeed
           << endl;
    }

    pos->SetSpeed(linSpeed, rotSpeed, 0);
    updateHolonomicVelocityStatistics();
  }
  else{
    calculateNonHolonomicSpeed();
    
    // suppress maximum velocity
    if (linSpeed > maxLinSpeed) linSpeed = maxLinSpeed;
    
    pos->SetXSpeed(linSpeed);
    pos->SetTurnSpeed(rotSpeed);
    updateNonHolonomicVelocityStatistics();
  }
}

// Change a vector to a fixed module, keeping scale.
void Robot::setConstantModule(double &x, double &y, double limit)
{
  double m = hypot(x,y);
  x = limit*x/m;
  y = limit*y/m;
}

//Used for get the angle of some laser beam on respect to the orientation of robot
double Robot::getBearing(int i)
{
  return dtor(-LASER_FOV/2 + (LASER_FOV/(LASER_SAMPLES-1))*i);
}

//Reads a configuration file and calculate the static parameters only once. After this procedure, cf is updated if was not initialised.
void Robot::readConfigFile(ConfigFile *cf)
{
  if (Robot::alreadyConfigurated) return;
  try{
    d            = atof(cf->valueOf("d").c_str());
    D            = atof(cf->valueOf("D").c_str());
    waypointDist = atof(cf->valueOf("s").c_str());
    folder       = cf->valueOf("folder");
    log_name     = cf->valueOf("log");
    isHolonomic  = (atoi(cf->valueOf("holo").c_str()) != 0);
    maxLinSpeed  = atof(cf->valueOf("v").c_str());
  }
  catch (string s) {
    cout << endl << "Robot.o: Configuration file is incorret: " << s << endl;
    exit(1);
  }
  try{
    testTime = atoll(cf->valueOf("time").c_str());
  }
  catch (string s) {
    testTime = maxTestTime;
  }
  try{
    savePosVel = (atoi(cf->valueOf("saveVelocities").c_str()) != 0);
  }
  catch (string s) {
    savePosVel = false;
  }
  try{
    saveArrivedLeftRobots = (atoi(cf->valueOf("saveArrivedLeftRobots").c_str()) != 0);
  }
  catch (string s) {
    saveArrivedLeftRobots = false;
  }
  unsigned int n;
  try{
    saveEnergy = (atoi(cf->valueOf("saveEnergy").c_str()) != 0);
  }
  catch (string s) {
    saveEnergy = false;
  }
  if (saveEnergy){
    try{
      n = atoi(cf->valueOf("n").c_str());
    }
    catch (string s){
      cout << endl << "Robot.o: Configuration file is incorret: " << s << endl;
      exit(1);
    }
    energyWasSummed = new vector<bool>(n);
    for (unsigned int i = 0; i < n; i++) (*energyWasSummed)[i] = false;
    instantTotalEnergy = 0;
    logE.open((folder+"/"+log_name+"E").c_str());
  }
  if (saveArrivedLeftRobots){
    arrivedLeftLog.open((folder+"/LogArrivedAndLeft").c_str());
  }
  Robot::minDistance = d;
  Robot::alreadyConfigurated = true;
}

//Verify how many times robot stall.
void Robot::checkStall() {
  if (pos->Stalled()) {
    if (!alreadyStalled) {
      stalls++;
      alreadyStalled = true;
    }
  } else {
    alreadyStalled = false;
  }
}

// Obtains current robot's position and updates mx, my and m_th.
void Robot::getPosition() {
  Pose pose = this->pos->GetPose();
  m_x = pose.x;
  m_y = pose.y;
  m_th = pose.a;
  #ifdef DEBUG_FORCES
    fv.setPosition(m_x,m_y,m_th);
  #endif
}

// Check if robot has finished and saves log if it has. Waits a bit for changing to finished state and to saves log depending on ITERATION_FOR_CHANGING_COLOR in commonConfig.h
void Robot::saveLogAndWaitIfFinished(){
  if (finished && exitedFromTargetRegion)
  {
    // This condition serves to give some time before finishing simulation. 
    //If ITERATION_FOR_CHANGING_COLOR is 1, the robot changes immediately to finished state and saves the log. This is used for generate videos or plotting images, so that the last robot will be black. Use ITERATION_FOR_CHANGING_COLOR = 99 in commonConfig.h when do that. 
    if (alreadyChanged % ITERATION_FOR_CHANGING_COLOR == 0){
      FinalLog::finish();
    }
    alreadyChanged++;
  }
}

// checks if the time limit was reached. The time limit can be configurated by .ini file. If it is not set, the default value in commonConfig.h is used.
void Robot::checkTimeIsOver(){
  if ((testTime - FINISH_TIME < theWorld->SimTimeNow()) && !finishedBySimTime )
  {
    finishedBySimTime = true;
    numIterationsReachGoal = numIterations;
    numIterations = 0;
    FinalLog::refresh_not_at_target(numIterationsReachGoal, numIterations, stalls, maxVelocity, minDistance);
    FinalLog::notify_finish_not_at_target();
    finish();
    FinalLog::finish();
  }
}

// Set new destination when changed.
void Robot::setDestination(){
  destinationX = waypoints[currentWaypoint][0];
  destinationY = waypoints[currentWaypoint][1];
}

// Calculate the attractive force to point (gotoX,gotoY).
// The modulo of the force will be Ka.
// Return the result to Fx and Fy.
void Robot::calculateAttractiveForce(double &Fx, double &Fy, double m_x, double m_y, double gotoX, double gotoY)
{
  Fx = (gotoX - m_x);
  Fy = (gotoY - m_y);
  setConstantModule(Fx,Fy,Ka);
}

// Set fx and fy to the attractive force to point (gotoX,gotoY).
// The modulo of (fx,fy) will be Ka. Modifies the variables fx and fy.
void Robot::setAttractiveForce(double& fx, double& fy, double m_x, double m_y, double gotoX, double gotoY
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  calculateAttractiveForce(fx, fy, m_x, m_y, gotoX, gotoY);
  #ifdef DEBUG_FORCES
    fv.setAttractiveForces(fx,fy);
  #endif
}

//Alters the values of fx and fy, adding repulsion force. Also, it updates the mean and variance of distances between robots sensed by laser ranges.
void Robot::obstaclesRepulsionForces()
{  
   double distance;
   double dx = 0;
   double dy = 0;
   #ifdef DEBUG_FORCES
     double fx_ = 0 ,fy_ = 0;
   #endif

   const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
   uint32_t sample_count = scan.size();
   
   double influence = d;
   double min_scan = influence;
   for(uint i = 0; i < sample_count; i++)
   {
      distance = scan[i];
      min_scan = min(distance, min_scan);
      if (distance <= influence)
      {
         if (distance <= minDistance)
            minDistance = distance;
        
         dx = distance*cos(m_th + getBearing(i));
         dy = distance*sin(m_th + getBearing(i));

         double _fx = -Kobs*(1.0/distance - 1.0/influence)*(1.0/pow((double)distance,2))*(dx/distance);
         double _fy = -Kobs*(1.0/distance - 1.0/influence)*(1.0/pow((double)distance,2))*(dy/distance);

         fx += _fx;
         fy += _fy;
         #ifdef DEBUG_FORCES
           fx_ += _fx;
           fy_ += _fy;
         #endif
      }
   }
   if (!finished){
     n_distances++;
     IterativeMeanVariance(min_scan, n_distances, &mean_distance, &var_distance);
   }
   #ifdef DEBUG_FORCES
     fv.setRepulsiveForces(fx_,fy_);
   #endif
}

// Updates the log with leaving data and change robot's state when leaving the the algorithm working range.
void Robot::updateLogWhenLeavingTarget(bool additionalCondition, States leavingState, Color const& color){
  if (additionalCondition && hypot(m_x - waypoints[0][0], m_y - waypoints[0][1]) > D)
  {
    if (leavingState != INVALID_STATE) m_state = leavingState;
    pos->SetColor(color);
    FinalLog::refresh(numIterationsReachGoal, numIterations, stalls, theWorld->SimTimeNow(), reachingTargetTime, maxVelocity, minDistance, mean_distance, var_distance, n_distances, mean_velocity, var_velocity, n_velocities);
    FinalLog::notify_finish();
    exitedFromTargetRegion = true;
    finish();
    if (saveArrivedLeftRobots) writeInArrivedLeftLog(theWorld,m_id,false);
    if (leavingState == INVALID_STATE){
      FinalLog::finish();
      finished = false;
    }
  }
}

// If the robot arrives in the target region, sets new target and save the spent time.
void Robot::setNewDestinationIfOnTarget(bool& finished, States& m_state, unsigned int & currentWaypoint, World* theWorld, unsigned int &numIterationsReachGoal, unsigned int &numIterations, Stg::usec_t &reachingTargetTime, ModelPosition* pos, States st, Color const& color, double m_x, double m_y, double destinationX, double destinationY, unsigned int m_id){
  if ((hypot(m_x - destinationX, m_y - destinationY) < waypointDist + epsilon) && !finished)
  {
    finished = true;
    m_state = st;
    if (pos != NULL) pos->SetColor(color);
    currentWaypoint = 1 + (rand() % NUMBER_OF_WAYPOINTS);
    numIterationsReachGoal = numIterations;
    numIterations = 0;
    reachingTargetTime = theWorld->SimTimeNow();
    if (saveArrivedLeftRobots) writeInArrivedLeftLog(theWorld,m_id,true);
  }
}

// calculate sum of potential and kinetic energy.
double Robot::calculateEnergy(double v, double vi){
  double Edissip, Ek, Ev = (v*v)/2, Evi = vi*vi/2;
  double EP, UA = Ka*hypot(m_x - destinationX, m_y - destinationY);
  double UR = 0;
  const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  double influence = d;
  for(uint i = 0; i < sample_count; i++)
  {
    if (scan[i] <= influence)
    {
       UR += -0.5*Kobs*(1.0/pow((1.0/scan[i] - 1.0/influence),2));
    }
  }
  EP = UA + UR;
  Edissip = U0 - EP - Ev;
  Ek = Ev + Edissip;
  logE << setfill(' ') << setw(9);
  logE << theWorld->SimTimeNow();
  logE << ", " << m_id << ":";
  logE.precision(6);
  logE.setf(ios::fixed);
  logE << " vo=" << v << endl; 
  
  logE << setfill(' ') << setw(9);
  logE << theWorld->SimTimeNow();
  logE << ", " << m_id << ":";
  logE.precision(6);
  logE.setf(ios::fixed);
  logE << " Evi=" << Evi << " UA=" << UA << " U0-UA=" << U0-UA << " U0=" << U0 << " Evi+UA=" << Evi+UA <<endl ;
    
       //~ << " Ed=" << Edissip
       //~ << " Ev=" << Ev
       //~ << " UA=" << UA
       //~ << " vE=" << sqrt(2*(U0-UA))
       //~ << " U0-UA=" << U0-UA 
  return Ek + EP;
}

// calculate sum of the energy of all robots.
void Robot::sumInstantEnergy(unsigned int id, double energy, World* theWorld){
  (*energyWasSummed)[id] = true;
  instantTotalEnergy += energy;
  bool allTrue = true;
  for (unsigned int i = 0; i < energyWasSummed->size(); i++){
    if (!(*energyWasSummed)[i]){
      allTrue = false;
      break;
    }
  } 
  if (allTrue){
    logE << theWorld->SimTimeNow() << ": " << instantTotalEnergy << endl;
    instantTotalEnergy = 0;
    for (unsigned int i = 0; i < energyWasSummed->size(); i++) 
      (*energyWasSummed)[i] = false;
  }
}
