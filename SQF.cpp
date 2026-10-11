#include "SQF.h"

SQF::SQF(ConfigFile *confFile) : Robot(confFile) {}

//Initialize robot data for SQF algorithm.
void SQF::init(int id)
{
  Robot::init(id);
  m_state = GOING;
  pos->SetColor(GOING_COLOR);
}

// Calculate rotation force field for entering 
void SQF::calculateEnteringRotationForce(double& fx, double& fy, double m_x, double m_y, double destinationX, double destinationY
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  if (m_x - destinationX > 0)
  {
    fx = - (m_y - destinationY);
    fy =   (m_x - destinationX);
  }
  else
  {
    fx =   (m_y - destinationY);
    fy = - (m_x - destinationX);
  }
  setConstantModule(fx,fy,Ka);
  #ifdef DEBUG_FORCES
    fv.setAttractiveForces(fx,fy);
  #endif
}

// Calculate rotation force field for leaving 
void SQF::calculateLeavingRotationForce(double& fx, double& fy, double m_x, double m_y, double destinationX, double destinationY, double D
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  if (destinationX > waypoints[0][0])
  {
    fx = - (m_y - waypoints[0][1]);
    fy =   (m_x - (waypoints[0][0] + D));
  }
  else
  {
    fx =   (m_y - waypoints[0][1]);
    fy = - (m_x - (waypoints[0][0] - D));
  
  }
  setConstantModule(fx,fy,Ka);
  #ifdef DEBUG_FORCES
    fv.setAttractiveForces(fx,fy);
  #endif
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void SQF::mainLoop(){
  numIterations++;
  checkStall();
  getPosition();
  saveLogAndWaitIfFinished();
  setNewDestinationIfOnTarget(finished, m_state, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, pos, GOING_OUT, GOING_OUT_COLOR, m_x, m_y, destinationX, destinationY, m_id);
  updateLogWhenLeavingTarget(finished, INVALID_STATE, END_COLOR);
  checkTimeIsOver();
  setDestination();

  calculateStateDependentForce(fx, fy, m_state, pos, m_x, m_y, m_th, destinationX, destinationY
  #ifdef DEBUG_FORCES
    , fv
  #endif
  );
  SQF::obstaclesRepulsionForces(fx, fy, &mean_distance, &var_distance, n_distances, laser, finished, m_state, m_x, m_y, m_th, destinationX, destinationY
  #ifdef DEBUG_FORCES
  , fv
  #endif
  );
  setConstantModule(fx, fy, maxForce);
  #ifdef DEBUG_FORCES
  fv.setResultantForces(fx,fy);
  #endif
  setSpeeds();
}

// Calculate the TRVF force depending on the robot's state
void SQF::calculateStateDependentForce(double& fx, double& fy, States &m_state, ModelPosition* pos, double m_x, double m_y, double m_th, double destinationX, double destinationY
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
{
  if (m_state != GOING_OUT)
  {
    if (hypot(m_x - destinationX, m_y - destinationY) < D && (m_y < destinationY || abs(m_x - destinationX) > waypointDist))
    {
      m_state = WAIT;
      if (pos != NULL) pos->SetColor(WAIT_COLOR);
      calculateEnteringRotationForce(fx, fy, m_x, m_y, destinationX, destinationY
      #ifdef DEBUG_FORCES
        , fv
      #endif
      );
    }
    else
    {
      m_state = GOING;
      if (pos != NULL) pos->SetColor(GOING_COLOR);
      setAttractiveForce(fx,fy,m_x,m_y,destinationX,destinationY
      #ifdef DEBUG_FORCES
        , fv
      #endif
      );
    }
  }
  else // m_state == GOING_OUT
  {
    if ( hypot(m_x - waypoints[0][0], m_y - waypoints[0][1]) < D)
      calculateLeavingRotationForce(fx, fy, m_x, m_y, destinationX, destinationY, D
      #ifdef DEBUG_FORCES
        , fv
      #endif
      );
    else
      setAttractiveForce(fx,fy,m_x,m_y,destinationX,destinationY
      #ifdef DEBUG_FORCES
        , fv
      #endif
      );
  }
}

//Alters the values of fx and fy, adding repulsion force. Also, it updates the mean and variance of distances between robots sensed by laser ranges.
void SQF::obstaclesRepulsionForces(double &fx, double &fy, double* mean_distance, double* var_distance, unsigned long &n_distances, ModelRanger* laser, bool finished, States m_state, double m_x, double m_y, double m_th, double destinationX, double destinationY
#ifdef DEBUG_FORCES
  , ForceVisualizer& fv
#endif
)
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
   const double Imin = 1.;
   double min_scan = influence;
   if (m_state == GOING || 
       (m_state == GOING_OUT && 
        finished))
   {
     influence = Imin;
   }
   else if ((m_state == WAIT) && (m_y > destinationY) && (fabs(m_x - destinationX) < d - Imin))
   {
     influence = Imin + fabs(m_x - destinationX);
   }
   for(uint i = 0; i < sample_count; i++)
   {
      distance = scan[i];
      min_scan = min(distance, min_scan);
      if (distance <= influence)
      {
         if (distance <= minDistance && !finished)
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
     IterativeMeanVariance(min_scan, n_distances, mean_distance, var_distance);
   }
   #ifdef DEBUG_FORCES
     fv.setRepulsiveForces(fx_,fy_);
   #endif
}
