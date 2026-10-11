#ifndef _FINALLOG_H_
#define _FINALLOG_H_

#include "commonConfig.h"
#include "util.h"
#include <fstream>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <stage.hh>
#include <math.h>

using namespace std;

class FinalLog{
  private:
    static unsigned int num_robots,numFinished,numFinishedAtTarget;
    static bool initiated;
    static ofstream logFile;
    static unsigned int numTotalIterationsReachGoal; // Total number of iterations
    static unsigned int numTotalIterationsExitGoal; // Total number of iterations
    static unsigned int numMaxIterationsReachGoal; //Maximum number of iterations to reach the goal
    static unsigned int numMaxIterationsExitGoal; //Maximum number of iterations to exit from goal
    static unsigned int numTotalStalls; //number of times that the robots stalled
    static Stg::usec_t simTime; //simulation time in microseconds
    static Stg::usec_t minReachingTargetTime, maxReachingTargetTime; //max and min simulation time in microseconds to reach the target between all robots
    static double minDistance;  //minimum distance between robots detected by sensors for all robots
    static double maxVelocity; //maximum velocity achieved by any robot 
    static double meanDistance, varDistance;  //mean and variance of distance between robots detected by sensors for all robots
    static double meanVelocity, varVelocity; //mean and variance of velocity achieved by any robot 
    static unsigned long nDistance, nVelocity; //number of mean and variance of distance and velocity entered so far.
    static void saveLog();
    static string path;

  public:
    static void init(string path);
    static void refresh_not_at_target(unsigned int numIterationsReachGoal,
                               unsigned int numIterationsExitGoal,
                               unsigned int numStalls,
                               double maxVel,
                               double minDist);
                               
    static void refresh(unsigned int numIterationsReachGoal,
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
                        unsigned long nveloc);
                        
    static void notify_finish();
    static void finish();
    static void notify_finish_not_at_target();
};
#endif
