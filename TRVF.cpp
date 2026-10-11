#include "TRVF.h"

bool TRVF::alreadyConfigurated = false;
TRVFparameters TRVF::TRVFparam;

/* Constructor arguments:
    conf: file name with experiments variables.*/
TRVF::TRVF(ConfigFile *confFile) : Robot(confFile)
{
  TRVF::readConfigFile(confFile);
}

//Static initialization of some TRVF trajectory variables
void TRVF::TRVFinit(TRVFtrajectory *TRVFtraj){
  TRVFtraj->line1parametersWasNotCalculated          =
  TRVFtraj->line2parametersWasNotCalculated          =
  TRVFtraj->toTargetRegionParametersWasNotCalculated =
  TRVFtraj->circularParametersWasNotCalculated       =
  TRVFtraj->circularParameters2WasNotCalculated      = true;
}

//Initialize robot data for TRVF algorithm.
void TRVF::init(int id)
{
  Robot::init(id);
  m_state = GOING;
  pos->SetColor(GOING_COLOR);
  TRVFinit(&TRVFtraj);
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void TRVF::mainLoop()
{  
  numIterations++;
  checkStall();
  getPosition();
  saveLogAndWaitIfFinished();
  updateLogWhenLeavingTarget(m_state == LEAVING_CURVE || m_state == LEAVING_LINE2, GOING_OUT, END_COLOR);
  checkTimeIsOver();
  setDestination();
  calculateStateDependentForce(fx, fy, m_state, finished, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, &TRVFparam, &TRVFtraj, pos, m_x, m_y, m_th, destinationX, destinationY, m_id
  #ifdef DEBUG_FORCES
  , fv
  #endif
  );
  obstaclesRepulsionForces();
  setConstantModule(fx, fy, maxForce);
  #ifdef DEBUG_FORCES
    fv.setResultantForces(fx,fy);
  #endif
  setSpeeds();
}

// Calculate the TRVF force depending on the robot's state
void TRVF::calculateStateDependentForce(double& fx, double& fy, States &m_state, bool &finished, unsigned int & currentWaypoint, World* theWorld, unsigned int &numIterationsReachGoal, unsigned int &numIterations, Stg::usec_t &reachingTargetTime, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj, ModelPosition* pos, double m_x, double m_y, double m_th, double destinationX, double destinationY, unsigned int m_id
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  if (m_state == GOING && hypot(m_x - destinationX, m_y - destinationY) <= D){
    m_state = ENTERING_OUTSIDE;
    if (pos != NULL) pos->SetColor(ENTERING_OUTSIDE_COLOR);
  }
  if (m_state == GOING_OUT || m_state == GOING){
    setAttractiveForce(fx, fy, m_x, m_y, destinationX, destinationY
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
  }
  else{
    coolPathFollowing(fx, fy, m_state, finished, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, TRVFparam, TRVFtraj, m_x, m_y, m_th, destinationX, destinationY, m_id
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
    if (pos != NULL) changeColor(m_state, pos);
  }
  if (m_state == GOING_OUT){
    targetRegionRepulsionForce(fx,fy,m_x,m_y
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
  }
}

//Change the color of the robot based on its state.
void TRVF::changeColor(States m_state, ModelPosition* pos){
  if (m_state == ENTERING_LINE1)
    pos->SetColor(ENTERING_LINE1_COLOR);
  else if (m_state == ENTERING_CURVE)
    pos->SetColor(ENTERING_CURVE_COLOR);
  else if (m_state == LEAVING_CURVE)
    pos->SetColor(LEAVING_CURVE_COLOR);
  else if (m_state == LEAVING_LINE2)
    pos->SetColor(LEAVING_LINE2_COLOR);
}

// Add to fx and fy the attractive force from (m_x,m_y) to point (gotoX,gotoY).
// Modifies the parameters fx and fy.
// The modulo of the force after the addition will be Ka.
void TRVF::addAttractiveForce(double& fx, double& fy, double gotoX, double gotoY, double m_x, double m_y
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  double _fx = (gotoX - m_x);
  double _fy = (gotoY - m_y);
  // This constant must be greater than the one used on the circular path to avoid local minima on the line and circle paths' joining.
  setConstantModule(_fx,_fy,1.5*Ka);
  fx += _fx;
  fy += _fy;
  setConstantModule(fx,fy,Ka);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
}

/*Set line 1 parameters just once.
  Arguments:
    lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
    TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj: parameters of the TRVF trajectory.*/
inline void TRVF::setLine1PathParameters(double lineangle, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj)
{
  if (TRVFtraj->line1parametersWasNotCalculated){
    double ca = cos(lineangle);
    double sa = sin(lineangle);
    double d2sa = (d/2)*sa;
    double d2ca = (d/2)*ca;
    /*
    Corridor variables map:
    
     (X1-d2sa,Y1+d2ca)          (X2-d2sa,Y2+d2ca)
              __________________________
             |                          |
     (X1,Y1) --------------------------- (X2,Y2)
             |__________________________|
     (X1+d2sa,Y1-d2ca)          (X2+d2sa,Y2-d2ca)
    */
    double X1 = waypoints[0][0] + TRVFparam->dcurve*ca;
    double Y1 = waypoints[0][1] + TRVFparam->dcurve*sa;
    double X2 = waypoints[0][0] + (TRVFparam->dcurve + TRVFparam->corridorLength)*ca;
    double Y2 = waypoints[0][1] + (TRVFparam->dcurve + TRVFparam->corridorLength)*sa;
    //setting the parameters used by the vector field generator
    //starting point
    TRVFtraj->w1x = X2 + d2sa;
    TRVFtraj->w1y = Y2 - d2ca;
    //ending point
    TRVFtraj->w2x = X1 + d2sa;  
    TRVFtraj->w2y = Y1 - d2ca;
    // vector w2 - w1 and its modulus
    TRVFtraj->w2_w1x = TRVFtraj->w2x - TRVFtraj->w1x;
    TRVFtraj->w2_w1y = TRVFtraj->w2y - TRVFtraj->w1y;
    double w2_w1mod = hypot(TRVFtraj->w2_w1x, TRVFtraj->w2_w1y);
    TRVFtraj->w2_w1mod2 = w2_w1mod*w2_w1mod;
    TRVFtraj->line1parametersWasNotCalculated = false;
  }
}

/* Set line 2 parameters just once.
  Arguments:
    lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
    TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj: parameters of the TRVF trajectory. */
inline void TRVF::setLine2PathParameters(double lineangle, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj)
{
  //Calculate just the exiting waypoint
  double w4x,w4y;
  if (TRVFtraj->line2parametersWasNotCalculated){
    double ca = cos(lineangle-TRVFparam->alpha);
    double sa = sin(lineangle-TRVFparam->alpha);
    double d2sa = (d/2)*sa;
    double d2ca = (d/2)*ca;
    /*
    Corridor variables map:
    
     (X1-d2sa,Y1+d2ca)          (X2-d2sa,Y2+d2ca)
              __________________________
             |                          |
     (X1,Y1) --------------------------- (X2,Y2)
             |__________________________|
     (X1+d2sa,Y1-d2ca)          (X2+d2sa,Y2-d2ca)
    */
    double X1 = waypoints[0][0] + TRVFparam->dcurve*ca;
    double Y1 = waypoints[0][1] + TRVFparam->dcurve*sa;
    double X2 = waypoints[0][0] + (TRVFparam->dcurve + TRVFparam->corridorLength)*ca;
    double Y2 = waypoints[0][1] + (TRVFparam->dcurve + TRVFparam->corridorLength)*sa;
    //starting point
    TRVFtraj->w3x = X1 - d2sa;
    TRVFtraj->w3y = Y1 + d2ca;
    //ending point
    w4x = X2 - d2sa;
    w4y = Y2 + d2ca;
    // vector w2 - w1 and its modulus
    TRVFtraj->w4_w3x = w4x - TRVFtraj->w3x;
    TRVFtraj->w4_w3y = w4y - TRVFtraj->w3y;
    double w4_w3mod = hypot(TRVFtraj->w4_w3x, TRVFtraj->w4_w3y);
    TRVFtraj->w4_w3mod2 = w4_w3mod*w4_w3mod;
    TRVFtraj->line2parametersWasNotCalculated = false;
  }
}

/* Line path vector field constructor. Sets fx and fy. Let w1 be the line starting point coordinates and w2 the line ending point coordinates in the arguments explanations below.
Arguments:
  fx, fy: force vector. Modifies fx and fy after execution.
  w2_w1x_, w2_w1y_, w2_w1mod2_: x and y coordinates of the vector w2 - w1 and its modulus squared;
  w1x_, w1y_: x and y coordinates of the vector w1.
  powtauk: parameter for the exponent in the rotational controller
  m_x, m_y, m_th: robot's pose.
Return true if the robot arrived next to the ending waypoint w2.*/
bool TRVF::linePathFollowing(double& fx, double& fy, double w2_w1x_, double w2_w1y_, double w2_w1mod2_, double w1x_, double w1y_, double powtauk, double m_x, double m_y, double m_th
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  double z_w1x = m_x - w1x_, z_w1y = m_y - w1y_;
  double s = (z_w1x*w2_w1x_ + z_w1y*w2_w1y_)/w2_w1mod2_;
  double Xc;
  if (s >= 1){
    return true;
  }
  else{
    double Xf = atan2(w2_w1y_, w2_w1x_);
    double ep = hypot(z_w1x - s*w2_w1x_, z_w1y - s*w2_w1y_);
    int rho = (w2_w1x_*z_w1y - w2_w1y_*z_w1x >= 0)? 1 : -1 ;
    const double tau = d/5, Xe = PI/2;
    if (ep > tau){
      Xc = Xf - rho*Xe;
    }
    else{
      ep *= rho;
      double power1 = pow(ep/tau,Kexpoent1), power2 = pow(ep,Kexpoent1-1);
      if (isnan(power1)) power1 = 0;
      if (isnan(power2)) power2 = 0;
      double Xd = Xf - Xe*power1;
      Xc = Xd - ((Kexpoent1*Xe*maxLinSpeed)/(Kr*powtauk))*power2*sin(m_th);
    }
  }
  fx = Ka*cos(Xc);
  fy = Ka*sin(Xc);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
  return false;
}


/* Set first circular path parameters just once.*/
inline void TRVF::setCircularPathParameters(TRVFtrajectory *TRVFtraj)
{
  if (TRVFtraj->circularParametersWasNotCalculated){
    TRVFtraj->cx = waypoints[0][0];
    TRVFtraj->cy = waypoints[0][1];
    //w1 - c vector
    TRVFtraj->w1cx = TRVFtraj->w1x - TRVFtraj->cx;
    TRVFtraj->w1cy = TRVFtraj->w1y - TRVFtraj->cy;
    TRVFtraj->r1 = D;
    TRVFtraj->powrk = pow(TRVFtraj->r1,Kexpoent2);
    TRVFtraj->circularParametersWasNotCalculated = false;
  }
}

/* Set circular parameters for second circular path just once. 
Arguments:
  lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
  TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj: parameters of the TRVF trajectory. */
inline void TRVF::setCircularPathParameters2(int linenumber, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj)
{
  if (TRVFtraj->circularParameters2WasNotCalculated){
    double ang = (linenumber - 0.5)*TRVFparam->alpha;
    double cb = cos(ang);
    double sb = sin(ang);
    TRVFtraj->cx = waypoints[0][0] + (waypointDist+TRVFparam->r)*cb; 
    TRVFtraj->cy = waypoints[0][1] + (waypointDist+TRVFparam->r)*sb;
    TRVFtraj->wcx = TRVFtraj->cx - TRVFparam->r*cb;
    TRVFtraj->wcy = TRVFtraj->cy - TRVFparam->r*sb;
    //w1 - c vector
    TRVFtraj->w1cx = TRVFtraj->w3x - TRVFtraj->cx;
    TRVFtraj->w1cy = TRVFtraj->w3y - TRVFtraj->cy;
    TRVFtraj->r1 = TRVFparam->r;
    TRVFtraj->powrk = pow(TRVFtraj->r1,Kexpoent2);
    TRVFtraj->circularParameters2WasNotCalculated = false;
  }
}

/* Cicular path vector field constructor. Modifies the parameters fx and fy. 
Returns a float value: <= 0 and >= 1 indicates the robot already reached the final waypoint. */
double TRVF::circularPathFolowing(double &fx, double &fy, TRVFtrajectory *TRVFtraj, double m_x, double m_y, double m_th
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  double dx = m_x - TRVFtraj->cx, dy = m_y - TRVFtraj->cy, dm = hypot(dx, dy);
  double Xc, s = dx*TRVFtraj->w1cy -  dy*TRVFtraj->w1cx;
  if (s <= 0 ) return s;
  double gamma = atan2(dx,dy);
  if (dm > 2*TRVFtraj->r1){
    Xc = gamma - 5*PI/6 + (maxLinSpeed/dm)*sin(m_th - gamma);
  }
  else{
    double power1,power2;
    power1 = pow((dm - TRVFtraj->r1)/TRVFtraj->r1,Kexpoent2);
    power2 = pow(dm - TRVFtraj->r1,Kexpoent2-1);
    if (isnan(power1)) power1 = 0;
    if (isnan(power2)) power2 = 0;
    double Xd = gamma - PI/2. - (PI/3.)*power1;
    Xc = Xd - (maxLinSpeed*sin(m_th - gamma))/(Kr*dm) - (Kexpoent2*maxLinSpeed*PI*power2*cos(m_th-gamma))/(3*TRVFtraj->powrk*Kr);
  }
  /* Xc is oriented based on the vector (0,1) and clockwise in the original paper. To convert Xc to default orientation (i.e., based on vector (1,0) and counter-clockwise), use PI/2 - Xc.
  */
  fx = Ka*cos(PI/2-Xc);
  fy = Ka*sin(PI/2-Xc);
  #ifdef DEBUG_FORCES
  fv.setAttractiveForces(fx,fy);
  #endif
  return s;
}



/* Follow the line-circle-line vector field. Modifies fx, fy, m_state, finished, currentWaypoint,  pos, theWorld, numIterationsReachGoal, numIterations and reachingTargetTime.
Reference: Vector field path following for small unmanned air vehicles.
Conference Paper in Proceedings of the American Control Conference · July 2006
DOI: 10.1109/ACC.2006.1657648 */
void TRVF::coolPathFollowing(double& fx, double& fy, States &m_state, bool &finished, unsigned int & currentWaypoint, World* theWorld, unsigned int &numIterationsReachGoal, unsigned int &numIterations, Stg::usec_t &reachingTargetTime, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj, double m_x, double m_y, double m_th, double destinationX, double destinationY, unsigned int m_id
#ifdef DEBUG_FORCES
  ,ForceVisualizer& fv
#endif
)
{
  //Calculate the target direction vector angle.
  double ang = atan2(m_y - waypoints[0][1], m_x - waypoints[0][0]);
  if (ang < 0) ang = ang + 2*PI;
  
  //Get the entering lane angle next to ang.
  unsigned int linenumber = (unsigned int) floor(ang/TRVFparam->alpha) + 1; 
  double lineangle = linenumber*TRVFparam->alpha;


  if (m_state == ENTERING_OUTSIDE){
    setLine1PathParameters(lineangle, TRVFparam, TRVFtraj);
    setCircularPathParameters(TRVFtraj);
    if( circularPathFolowing(fx, fy, TRVFtraj, m_x, m_y, m_th
        #ifdef DEBUG_FORCES
          , fv
        #endif
        )
        <= 0
      )
    {
      m_state = ENTERING_LINE1;
    }
  }
  else if (m_state == ENTERING_LINE1){
    if ( linePathFollowing(fx, fy, TRVFtraj->w2_w1x, TRVFtraj->w2_w1y, TRVFtraj->w2_w1mod2, TRVFtraj->w1x, TRVFtraj->w1y, TRVFparam->powtauk, m_x, m_y, m_th
         #ifdef DEBUG_FORCES
           , fv
         #endif
         )
       )
    {
      m_state = ENTERING_CURVE;
    }
  }
  else if (m_state == ENTERING_CURVE){
    setLine2PathParameters(lineangle, TRVFparam, TRVFtraj);
    setCircularPathParameters2(linenumber, TRVFparam, TRVFtraj);
    circularPathFolowing(fx, fy, TRVFtraj, m_x, m_y, m_th
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
    addAttractiveForce(fx, fy, waypoints[0][0], waypoints[0][1], m_x, m_y
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
    setNewDestinationIfOnTarget(finished, m_state, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, NULL, LEAVING_CURVE, LEAVING_CURVE_COLOR, m_x, m_y, destinationX, destinationY, m_id);
  }
  else if (m_state == LEAVING_CURVE){
    double  ended = circularPathFolowing(fx, fy, TRVFtraj, m_x, m_y, m_th 
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
    addAttractiveForce(fx, fy, TRVFtraj->w3x, TRVFtraj->w3y, m_x, m_y
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
    if (
      ended <= 0 
      /* the condition below is for the case when K is greater than the maximum allowed value for r > 0. In these cases, there are paths to target region, but the curve is reversed, leading to errors. */
      || hypot(m_x - TRVFtraj->w3x, m_y - TRVFtraj->w3y) <= epsilonLineFollowing ){
      m_state = LEAVING_LINE2;
    }
  }
  else if (m_state == LEAVING_LINE2){
    linePathFollowing(fx, fy, TRVFtraj->w4_w3x, TRVFtraj->w4_w3y, TRVFtraj->w4_w3mod2, TRVFtraj->w3x, TRVFtraj->w3y, TRVFparam->powtauk, m_x, m_y, m_th
    #ifdef DEBUG_FORCES
      , fv
    #endif
    );
  }
}

//Alter the values of fx and fy, adding repulsion force by target region.
void TRVF::targetRegionRepulsionForce(double& fx, double& fy, double m_x, double m_y
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  double fx_ = 0 ,fy_ = 0;
  double influence =  D;
  
  double tzx = waypoints[0][0] - m_x, tzy = waypoints[0][1] - m_y;
  double tzm = hypot(tzx,tzy);
  double distance = tzm - D;
  if (distance <= influence)
  {
    if (distance > 0){
      fx_ = -Kobs*(1.0/distance - 1.0/influence)*(1.0/pow(distance,2))*(tzx/tzm);
      fy_ = -Kobs*(1.0/distance - 1.0/influence)*(1.0/pow(distance,2))*(tzy/tzm);
    }
    else{
      fx_ = -Kobs*tzm*tzx;
      fy_ = -Kobs*tzm*tzy;
    }
  }
  fx += fx_;
  fy += fy_;
  #ifdef DEBUG_FORCES
    fv.setTargetRepulsiveForces(fx_,fy_);
  #endif
}

//Reads a config file and calculate the static parameters.
void TRVF::readConfigFile(ConfigFile *cf)
{
  if (TRVF::alreadyConfigurated) return;
  try{
    TRVFparam.K_path = atoi(cf->valueOf("K").c_str());
  }
  catch (string s) {
    cout << endl << "TRVF.o: Configuration file is incorret: " << s << endl;
    exit(1);
  }
  TRVFparam.calculateParameters(waypointDist,d,D);
  TRVF::alreadyConfigurated = true;
}
