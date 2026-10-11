#include "NoCoord.h"

/* Constructor arguments:
    conf: file name with experiments variables.*/
NoCoord::NoCoord(ConfigFile *confFile) : Robot(confFile){ }

//Initialize robot data for no coordination algorithm.
void NoCoord::init(int id)
{
  Robot::init(id);
  m_state = GOING;
}

//Implements the main loop of robot. 
//Also contain robot controller and 
//probabilistic finite state machine codes
void NoCoord::mainLoop()
{  
  numIterations++;
  checkStall();
  getPosition();
  saveLogAndWaitIfFinished();
  setNewDestinationIfOnTarget(finished, m_state, currentWaypoint, theWorld, numIterationsReachGoal, numIterations, reachingTargetTime, pos, GOING, GOING_OUT_COLOR, m_x, m_y, destinationX, destinationY, m_id);
  updateLogWhenLeavingTarget(finished && m_state == GOING, GOING_OUT, END_COLOR);
  checkTimeIsOver();
  setDestination();
  setAttractiveForce(fx,fy,m_x,m_y,destinationX,destinationY
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
