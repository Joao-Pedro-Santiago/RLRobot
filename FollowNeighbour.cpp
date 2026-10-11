#include "FollowNeighbour.h"

bool FollowNeighbour::alreadyConfigurated = false;
double FollowNeighbour::minDotProductToFollow = 0.0;
Algorithms FollowNeighbour::alternativeAlgorithm;
TRVFparameters FollowNeighbour::TRVFparam;

/* Constructor arguments:
    conf: file name with experiments variables.*/
FollowNeighbour::FollowNeighbour(ConfigFile *confFile) : Robot(confFile)
{
  FollowNeighbour::readConfigFile(confFile);
}

//Initialize robot data for no coordination algorithm.
void FollowNeighbour::init(int id)
{
  Robot::init(id);
  m_state = GOING;
  pos->SetColor(GOING_ADHOC_COLOR);
  if (alternativeAlgorithm == TRVF_ALGORITHM){
    TRVF::TRVFinit(&TRVFtraj);
  }
  setAttractiveForce(fx, fy, m_x, m_y, destinationX, destinationY
  #ifdef DEBUG_FORCES
    , fv
  #endif
  );
}

// Check if there is any robot in neighbour and follow the one next to its current direction.
// If none is found, the robot executes the alternative algorithm.
void FollowNeighbour::checkNeighbourhoodAndFollow()
{
  const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  
  double GoalFx, GoalFy;
  calculateAttractiveForce(GoalFx, GoalFy, m_x, m_y, destinationX, destinationY);
  double influence = d, maxCosPrevForceLaserBeam = minDotProductToFollow, newfx = GoalFx, newfy = GoalFy, tmpfx = 0, tmpfy = 0;
  if (alternativeAlgorithm == NOCOORD_ALGORITHM){
    setNewDestinationIfOnTarget(finished, m_state, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, NULL, GOING, GOING_COLOR, m_x, m_y, destinationX, destinationY, m_id);
  }
  else if (alternativeAlgorithm == TRVF_ALGORITHM){
    TRVF::calculateStateDependentForce(newfx, newfy, m_state, finished, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, &TRVFparam, &TRVFtraj, NULL, m_x, m_y, m_th, destinationX, destinationY, m_id
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
  }
  else if (alternativeAlgorithm == SQF_ALGORITHM){
    setNewDestinationIfOnTarget(finished, m_state, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, NULL, GOING_OUT, GOING_OUT_COLOR, m_x, m_y, destinationX, destinationY, m_id);
    SQF::calculateStateDependentForce(newfx, newfy, m_state, NULL, m_x, m_y, m_th, destinationX, destinationY
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
    SQF::obstaclesRepulsionForces(newfx, newfy, &mean_distance, &var_distance, n_distances, laser, finished, m_state, m_x, m_y, m_th, destinationX, destinationY
    #ifdef DEBUG_FORCES
    , fv
    #endif
    );
  }
  
  for(uint i = 0; i < sample_count; i++)
  {
    double dx, dy, distance = scan[i];
    if (distance <= influence)
    {
      dx = cos(m_th + getBearing(i));
      dy = sin(m_th + getBearing(i));

      //Calculate cosine of the force and the laser beam direction with dot product, without considering the modulus.
      double dotPrevBeam = dx*newfx + dy*newfy;
      
      if (dotPrevBeam > maxCosPrevForceLaserBeam && dotPrevBeam > minDotProductToFollow) 
      {
        maxCosPrevForceLaserBeam = dotPrevBeam;
        tmpfx = Ka*dx;
        tmpfy = Ka*dy;
      }
    }
  }
  if (tmpfx != 0 || tmpfy != 0){
    fx = tmpfx;
    fy = tmpfy;
  }
  else{
    fx = newfx;
    fy = newfy;
  }
  #ifdef DEBUG_FORCES
    fv.setAttractiveForces(fx,fy);
  #endif
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void FollowNeighbour::mainLoop()
{  
  numIterations++;
  checkStall();
  getPosition();
  saveLogAndWaitIfFinished();
  
  updateLogWhenLeavingTarget(algorithmFinishingCondition(), algorithmFinalState(), END_ADHOC_COLOR);
  checkTimeIsOver();
  setDestination();
  checkNeighbourhoodAndFollow();
  // SQF as alternative algorithm has its own repulsive force calculation.
  if (alternativeAlgorithm != SQF_ALGORITHM)
    obstaclesRepulsionForces();
  setConstantModule(fx, fy, maxForce);
  #ifdef DEBUG_FORCES
  fv.setResultantForces(fx,fy);
  #endif
  setSpeeds();
}

//Change to the state of the experiment ending depending on the alternative algorithm.
inline States FollowNeighbour::algorithmFinalState(){
  if (alternativeAlgorithm == NOCOORD_ALGORITHM)
    return GOING_OUT;
  else if (alternativeAlgorithm == TRVF_ALGORITHM)
    return GOING_OUT;
  else if (alternativeAlgorithm == SQF_ALGORITHM)
    return GOING;
  else{
    cout << "Algorithm code invalid in FollowNeighbour::algorithmChangeFinalState()." << endl;
    exit(1);
  }
}

//Return the truth value of the condition to finish the experiment depending on the alternative algorithm
inline bool FollowNeighbour::algorithmFinishingCondition(){
  if (alternativeAlgorithm == NOCOORD_ALGORITHM)
    return finished && m_state == GOING;
  else if (alternativeAlgorithm == TRVF_ALGORITHM)
    return m_state == LEAVING_CURVE || m_state == LEAVING_LINE2;
  else if (alternativeAlgorithm == SQF_ALGORITHM)
    return m_state == GOING_OUT;
  else{
    cout << "Algorithm code invalid in FollowNeighbour::algorithmFinishingCondition()." << endl;
    exit(1);
  }
}


//Reads a configuration file and calculate the static parameters only once. After this procedure, cf is updated if was not initialised.
void FollowNeighbour::readConfigFile(ConfigFile *cf)
{
  if (FollowNeighbour::alreadyConfigurated) return;
  double angleDegrees;
  try{
    // angle in degrees to consider a valid neighbour to be followed, that is, the maximum absolute value of the angle between the target direction and the laser beam direction which detected a neighbour.
    angleDegrees = atof(cf->valueOf("neighbourhoodAngle").c_str());
    alternativeAlgorithm = strToAlgorithm(cf->valueOf("alternativeAlgorithm").c_str());
  }
  catch (string s) {
    //if none is given in the configuration file, 90 degrees is assumed.
    angleDegrees = 90.0;
  }
  if (alternativeAlgorithm == TRVF_ALGORITHM){
    try{
      TRVFparam.K_path = atoi(cf->valueOf("K").c_str());
    }
    catch (string s) {
      cout << endl << "Configuration file is incorret for TRVF alternative algorithm: " << s << endl;
      exit(1);
    }
    TRVFparam.calculateParameters(waypointDist,d,D);
  }
  minDotProductToFollow = cos(M_PI*angleDegrees/180.0)*Ka;
  FollowNeighbour::alreadyConfigurated = true;
}

//Convert an algorithm in C string to Algorithms code.
Algorithms FollowNeighbour::strToAlgorithm(const char* str){
  if (strcmp(str,"TRVF") == 0){
    return TRVF_ALGORITHM;
  }
  else if (strcmp(str,"NoCoord") == 0){
    return NOCOORD_ALGORITHM;
  }
  else if (strcmp(str,"SQF") == 0){
    return SQF_ALGORITHM;
  }
  else{
    cout << "Algorithm " << str << " not implemented." << endl;
    cout << "Available algorithms: " << endl
         << "TRVF NoCoord SQF." << endl;
    exit(1);
  }
}
