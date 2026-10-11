#include "FinalLog.h"


//alocating  statics members
unsigned int  FinalLog::num_robots, FinalLog::numFinished, FinalLog::numFinishedAtTarget;
bool  FinalLog::initiated;
ofstream  FinalLog::logFile;
unsigned int FinalLog::numTotalIterationsReachGoal; // Total number of iterations
unsigned int FinalLog::numTotalIterationsExitGoal; // Total number of iterations
unsigned int FinalLog::numMaxIterationsReachGoal; //Maximum number of iterations to reach the goal
unsigned int FinalLog::numMaxIterationsExitGoal; //Maximum number of iterations to exit from goal
unsigned int FinalLog::numTotalStalls; //number of times that the robots stalled
Stg::usec_t FinalLog::simTime; //simulation time in microseconds
string FinalLog::path;
double FinalLog::minDistance;  //minimum distance between robots detected by sensors for all robots
double FinalLog::maxVelocity; //maximum velocity achieved by any robot 
double FinalLog::meanDistance, FinalLog::varDistance;  //mean and variance of distance between robots detected by sensors for all robots
double FinalLog::meanVelocity, FinalLog::varVelocity; //mean and variance of velocity achieved by any robot 
unsigned long FinalLog::nDistance, FinalLog::nVelocity; //number of mean and variance of distance and velocity entered so far.
Stg::usec_t FinalLog::minReachingTargetTime, FinalLog::maxReachingTargetTime; //max and min simulation time in microseconds to reach the target between all robots

void FinalLog::init(string p){
  if (!initiated){
    initiated = true;
    simTime = 0;
    numTotalIterationsReachGoal = 0;
    numTotalIterationsExitGoal = 0;
    numMaxIterationsReachGoal = 0;
    numMaxIterationsExitGoal = 0;
    numTotalStalls = 0;
    numFinished = numFinishedAtTarget = 0;
    num_robots = 0;
    path = p;
    maxVelocity = 0;
    minDistance = 10000.;
    minReachingTargetTime = maxTestTime + FINISH_TIME + 1;
    maxReachingTargetTime = 0;
    nDistance = nVelocity = 0;
    varDistance = varVelocity = meanDistance = meanVelocity = 0;
  }
  num_robots++;
}

void FinalLog::refresh_not_at_target(unsigned int numIterationsReachGoal,
                                     unsigned int numIterationsExitGoal,
                                     unsigned int numStalls,
                                     double maxVel,
                                     double minDist){
  numTotalStalls += numStalls;
  numTotalIterationsReachGoal += numIterationsReachGoal;
  numTotalIterationsExitGoal += numIterationsExitGoal;
  if (numIterationsReachGoal > numMaxIterationsReachGoal)
     numMaxIterationsReachGoal = numIterationsReachGoal;
  if (numIterationsExitGoal > numMaxIterationsExitGoal)
     numMaxIterationsExitGoal = numIterationsExitGoal;
  if (maxVelocity < maxVel)
    maxVelocity = maxVel;
  if (minDist < minDistance)
    minDistance = minDist;
}

void FinalLog::refresh(unsigned int numIterationsReachGoal,
                    unsigned int numIterationsExitGoal,
                    unsigned int numStalls,
                    Stg::usec_t sim_time,
                    Stg::usec_t reaching_goal_time,
                    double maxVel,
                    double minDist,
                    double meanDist,
                    double varDist,
                    unsigned long ndist,
                    double meanVel, 
                    double varVel,
                    unsigned long nveloc){
  numTotalStalls += numStalls;
  numTotalIterationsReachGoal += numIterationsReachGoal;
  numTotalIterationsExitGoal += numIterationsExitGoal;
  if (numIterationsReachGoal > numMaxIterationsReachGoal)
     numMaxIterationsReachGoal = numIterationsReachGoal;
  if (numIterationsExitGoal > numMaxIterationsExitGoal)
     numMaxIterationsExitGoal = numIterationsExitGoal;
  if (sim_time > simTime)
    simTime = sim_time;
  if (maxVelocity < maxVel)
    maxVelocity = maxVel;
  if (minDist < minDistance)
    minDistance = minDist;
  if (reaching_goal_time > maxReachingTargetTime)
    maxReachingTargetTime = reaching_goal_time;
  if (reaching_goal_time < minReachingTargetTime)
    minReachingTargetTime = reaching_goal_time;
  varDistance = (nDistance*varDistance + ndist*varDist + ((meanDistance - meanDist)*(meanDistance - meanDist)*nDistance*ndist)/(nDistance + ndist))/(nDistance + ndist);
  meanDistance = (meanDistance*nDistance + meanDist*ndist)/(nDistance + ndist);
  nDistance+=ndist;
  varVelocity = (nVelocity*varVelocity + nveloc*varVel + ((meanVelocity - meanVel)*(meanVelocity - meanVel)*nVelocity*nveloc)/(nVelocity + nveloc))/(nVelocity + nveloc);
  meanVelocity = (meanVelocity*nVelocity + meanVel*nveloc)/(nVelocity + nveloc);
  nVelocity+=nveloc;
}

void FinalLog::saveLog(){
  logFile.open((path.c_str()));
  logFile << numTotalIterationsReachGoal + numTotalIterationsExitGoal << endl;
  logFile << numMaxIterationsReachGoal + numMaxIterationsExitGoal<< endl;
  logFile << 0 << endl; //This lines was added to match the other algorithms' logs.
  logFile << numTotalIterationsReachGoal << endl
          << numTotalIterationsExitGoal << endl;
  logFile << numMaxIterationsReachGoal << endl
          << numMaxIterationsExitGoal<< endl;
  logFile << numTotalStalls << endl;
  logFile << minReachingTargetTime << endl; //in microseconds
  logFile << maxReachingTargetTime << endl; //in microseconds
  logFile << simTime << endl; //in microseconds
  logFile << minDistance << endl;
  logFile << maxVelocity << endl;
  logFile << meanDistance << endl;
  logFile << sqrt(varDistance) << endl;
  logFile << nDistance << endl;
  logFile << meanVelocity << endl;
  logFile << sqrt(varVelocity) << endl;
  logFile << nVelocity << endl;
  logFile.close();
}

void FinalLog::notify_finish_not_at_target(){
  numFinished++;
  if (numFinished == num_robots){
     saveLog(); 
  }
}

void FinalLog::notify_finish(){
  numFinished++;
  numFinishedAtTarget++;
  if (numFinished == num_robots){
     saveLog(); 
  }
}

void FinalLog::finish(){
  if (numFinished == num_robots){
    exit(0);
  }
}
