/*
  This file implements a base robot to be inherited.
*/
#ifndef _ROBOT_H_
#define _ROBOT_H_


#include <stdlib.h>
#include <stage.hh>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include "FinalLog.h"
#include "ConfigFile.h"
#include "commonConfig.h"
#include "commonDefs.h"
#include "util.h"
#include "option.hh"
#ifdef DEBUG_FORCES
#include "forcevisualizer.h"
#endif

using namespace std;
using namespace Stg;

class Robot
{
  public:
    /* Constructor arguments:
    confFile: pointer to the ConfFile class with experiments variables.*/
    Robot(ConfigFile *confFile);
    
    //Initialize all robot data (pose, log file, velocity, etc.)
    virtual void init(int id);

    //Interface for the main loop of robot. 
    //Also contain robot controller and 
    //probabilistic finite state machine codes
    virtual void mainLoop() = 0;
    
    //Pointers to classes used in Stage
    ModelPosition* pos;
    ModelRanger* laser;
    World* theWorld; //ゴ ゴ ゴ ゴ
    
    //This member allows visualize forces for debug
    #ifdef DEBUG_FORCES
      ForceVisualizer fv;
    #endif
    
  protected:
    //Finishes a robot. Only displays a message for looking with the simulation is still running.
    void finish();
    
    // write in posVelLog the robot's velocity, position and acceleration. 
    void writeInPosVelLog(double x_acc, double y_acc);
    
    // write in posVelLog the robot's output velocity, position and acceleration. 
    void writeInPosOutVelLog(double linvec);
    
    // write in arrivedLeftLog when robot left or arrived.
    static void writeInArrivedLeftLog(World* theWorld, unsigned int m_id, bool arrived);
    
    //Subtract two angle values (in radians). The result value lies between -2 PI and 2 PI. 
    double angDiff(double end, double begin);
    
    //Initializes values for sensing with laser, depending on the world file used.
    void init_laser();
    
    /* Calculate x- and y-speed from force vector (fx,fy).
      Changes linSpeed and rotSpeed.
      Observation:
        linSpeed is repurposed to signify x_speed.
        rotSpeed is repurposed to signify y_speed. */
    void calculateHolonomicSpeed();
    
    /* Calculate x- and y-speed from force vector (fx,fy).
      Changes linSpeed and rotSpeed. */
    void calculateNonHolonomicSpeed();
    
    /* Updates maximum, mean and variance of velocity for holonomic robots */
    void updateHolonomicVelocityStatistics();
    
    /* Updates maximum, mean and variance of velocity for non-holonomic robots */
    void updateNonHolonomicVelocityStatistics();
    
    // Set linear and turning speeds based on the force vector. The linear velocity is set to a maximum value.
    void setSpeeds();
    
    // Change a vector to a fixed module, keeping scale.
    static void setConstantModule(double &x, double &y, double limit);

    //Used for get the angle of some laser beam on respect to the orientation of robot
    static double getBearing(int i);
    
    //Reads a config file and calculate the static parameters.
    static void readConfigFile(ConfigFile *cf);

    //Verify how many times robot stall.
    void checkStall();
    
    // Obtains current robot's position and updates mx, my and m_th.
    void getPosition();
    
    // Check if robot has finished and saves log if it has. Waits a bit for changing to finished state and to saves log depending on ITERATION_FOR_CHANGING_COLOR in commonConfig.h
    void saveLogAndWaitIfFinished();
    
    // checks if the time limit was reached. The time limit can be configurated by .ini file. If it is not set, the default value in commonConfig.h is used.
    void checkTimeIsOver();
    
    // Set new destination when changed.
    void setDestination();
    
    // Calculate the attractive force to point (gotoX,gotoY).
    // The modulo of the force will be Ka.
    // Return the result to Fx and Fy.
    static void calculateAttractiveForce(double &Fx, double &Fy, double m_x, double m_y, double gotoX, double gotoY);
    
    // Set fx and fy to the attractive force to point (gotoX,gotoY).
    // The modulo of (fx,fy) will be Ka. Modifies the variables fx and fy.
    static void setAttractiveForce(double& fx, double& fy, double m_x, double m_y, double gotoX, double gotoY
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    //Alters the values of fx and fy, adding repulsion force. Also, it updates the mean and variance of distances between robots sensed by laser ranges.
    void obstaclesRepulsionForces();
    
    // Updates the log with leaving data and change robot's state when leaving the the algorithm working range.
    void updateLogWhenLeavingTarget(bool additionalCondition, States leavingState, Color const& color);
    
    // If the robot arrives in the target region, sets new target and save the spent time.
    static void setNewDestinationIfOnTarget(bool& finished, States& m_state, unsigned int & currentWaypoint, World* theWorld, unsigned int &numIterationsReachGoal, unsigned int &numIterations, Stg::usec_t &reachingTargetTime, ModelPosition* pos, States st, Color const& color, double m_x, double m_y, double destinationX, double destinationY, unsigned int m_id);
    
    // calculate sum of potential and kinetic energy.
    double calculateEnergy(double v, double vi);
    
    // calculate sum of the energy of all robots.
    static void sumInstantEnergy(unsigned int id, double energy,  World* theWorld);
    
    //For checking if configuration was done yet.
    static bool alreadyConfigurated;

    //For logging maximum velocity and minimum distance
    static double maxVelocity, minDistance;
    
    //Radius of target area
    static double waypointDist;
    
    //Folder where the logs will be saved
    static string folder;
    
    //Log file name
    static string log_name;
    
    // Used for control holonomic or non-holonomic robots.
    static bool isHolonomic;
    
    // True if user wants to save the velocities and positions of the robots over the time in a log.
    static bool savePosVel;
    
    //Maximum time in miliseconds to test
    static unsigned long long testTime;

    //minimum distance between robots in corridor and minimum influence for repulsive force.
    static double d;

    //maximum distance from the target to the algorithm takes effect.
    static double D;
    
    //maximum linear velocity of the robot
    static double maxLinSpeed;

    //sum of the kinectic and potential energy of all robots at each instant. 
    static double instantTotalEnergy;
    
    // check the robots which have the energy summed.
    static vector<bool> *energyWasSummed;

    // if user wants to save total energy at each instant.
    static bool saveEnergy;
    
    // if user wants to save statistics when a robot has arrived at target and left the experiment region.
    static bool saveArrivedLeftRobots;

    // file for output the total energy.
    static ofstream logE;
    
    // file for the log of robots that arrived at the target and left the experiment region.
    static ofstream arrivedLeftLog;

    // for initial energy calculation
    double U0;

    // linear and turning speed for non-holonomic and x- and y-speed for holonomic.
    double linSpeed, rotSpeed;

    //The name of a robot, used in some functions of Player/Stage
    string m_name;
    
    //Actual x coordinate of robot position
    double m_x;
    
    //Actual y coordinate of robot position
    double m_y;
    
    //Current theta orientation of robot
    double m_th;

    //Resolution of execution for discrete movement controler
    const double TIME_STEP = 0.1;
    
    //Identifier of the robot. Used in communication and for generate the name of robot
    unsigned int m_id;

    //Current number of iterations.
    unsigned int numIterations;
    
    //number of iterations until reach the goal. The number of iterations from 
    //reach the goal until exit a specified area from goal will be the difference
    //between this value and numIterations
    unsigned int numIterationsReachGoal;

    //x and y coordinates to the goal position
    double destinationX, destinationY;

    // waypoints vector index where the robot are going
    unsigned int currentWaypoint;
    
    //State of robot in the experiment
    States m_state;

    //Indicates if the robot finished your execution
    bool finished;

    //Indicates if the robot finishes your execution by time limit
    bool finishedBySimTime;

    //Indicates if the robot exited from the target area
    bool exitedFromTargetRegion;
    
    //Used for changing the colour at the ending of the experiment
    int alreadyChanged;
    
    //FOV and SAMPLES for laser. These values are token from Stage world file and computed in init_laser()
    static double LASER_FOV,LASER_SAMPLES;
    
    //check if robot is already stalled
    bool alreadyStalled;
    
    //Number of times that robot became stalled
    unsigned int stalls;
    
    // time in microseconds to reach the goal.
    Stg::usec_t reachingTargetTime;
    
    // force vector for robot movement
    double fx,fy;
    
    //Variables for calculating mean and variance of distance and velocity;
    double mean_distance, var_distance, mean_velocity, var_velocity;
    unsigned long n_distances, n_velocities;
    
    ofstream posVelLog;
};

#endif
