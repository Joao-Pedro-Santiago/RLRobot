/*
  This file implements the TRVF algorithm.
*/
#ifndef _TRVF_H_
#define _TRVF_H_

#include "Robot.h"
#include "TRVFparameters.h"

struct TRVFtrajectory{
  /* Parameters for line trajectory 1. The line begins at w1 = (w1x,w1y) and ends at w2 = (w2x,w2y). 
  w2_w1x, w2_w1y, w2_w1mod: x and y coordinates of the vector w2 - w1 and its squared modulus;
  w1x, w1y: x and y coordinates of the vector w1.*/
  double w2_w1x, w2_w1y, w2_w1mod2, w1x, w1y;
  
  // Waypoint used on line trajectory 1 and circular trajectory 2.
  double w2x,w2y;
  
  //Centre of the circular trajectory
  double cx,cy;
  
  //starting point of the line trajectory 2.
  double w3x,w3y;

  // w1 - c's x and y coordinates. 
  double w1cx,w1cy;
  
  // waypoint in the intersection of the curved trajectory and the target region
  double wcx, wcy;
  
  //radius used on the circular trajectory vector field and value for circular path calculation.
  double r1, powrk;

  /*Parameters for line trajectory 2. The line begins at w3 = (w3x,w3y) and ends at w4 = (w4x,w2y). 
  w4_w3x, w4_w3y, w4_w3mod: x and y coordinates of the vector w4 - w3 and its squared modulus. */
  double w4_w3x,w4_w3y,w4_w3mod2;
  
  //For avoiding recalculation of the line 1, around target cicular path, towards target region path and line 2 parameters. They are false if the robot already calculate the respective path parameters.
  bool line1parametersWasNotCalculated, circularParametersWasNotCalculated, circularParameters2WasNotCalculated,toTargetRegionParametersWasNotCalculated, line2parametersWasNotCalculated;
};

class TRVF : public Robot
{
  public:
    /*Constructor arguments:
       conf: file name with experiments variables.
    */
    TRVF(ConfigFile* confFile);
    
    //Static initialization of some TRVF trajectory variables
    static void TRVFinit(TRVFtrajectory *TRVFtraj);
    
    //Initialize robot data for TRVF algorithm.
    void init(int id);
    
    //Implements the main loop of robot. 
    //Also contain robot controller and 
    //probabilistic finite state machine codes
    void mainLoop();

    // Calculate the TRVF force depending on the robot's state
    static void calculateStateDependentForce(double& fx, double& fy, States &m_state, bool &finished, unsigned int & currentWaypoint, World* theWorld, unsigned int &numIterationsReachGoal, unsigned int &numIterations, Stg::usec_t &reachingTargetTime, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj, ModelPosition* pos, double m_x, double m_y, double m_th, double destinationX, double destinationY, unsigned int m_id
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );

  protected:
    //Change the color of the robot based on its state.
    static void changeColor(States m_state, ModelPosition* pos);
    
    //Add to fx and fy the attractive force from (m_x,m_y) to point (gotoX,gotoY).
    //Modifies the parameters fx and fy.
    //The modulo of the force after the addition will be Ka.
    static void addAttractiveForce(double& fx, double& fy, double gotoX, double gotoY, double m_x, double m_y
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    /*Set line 1 parameters just once.
    Arguments:
      lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
      TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj: parameters of the TRVF trajectory.*/
    static inline void setLine1PathParameters(double lineangle, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj);

    /* Set line 2 parameters just once.
    Arguments:
      lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
      TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj: parameters of the TRVF trajectory. */
    static inline void setLine2PathParameters(double lineangle, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj);
    
    /* Line path vector field constructor. Sets fx and fy. Let w1 be the line starting point coordinates and w2 the line ending point coordinates in the arguments explanations below.
    Arguments:
      fx, fy: force vector. Modifies fx and fy after execution.
      w2_w1x_, w2_w1y_, w2_w1mod2_: x and y coordinates of the vector w2 - w1 and its modulus squared;
      w1x_, w1y_: x and y coordinates of the vector w1.
      powtauk: parameter for the exponent in the rotational controller
      m_x, m_y, m_th: robot's pose.
    Return true if the robot arrived next to the ending waypoint w2.*/
    static bool linePathFollowing(double& fx, double& fy, double w2_w1x_, double w2_w1y_, double w2_w1mod2_, double w1x_, double w1y_, double powtauk, double m_x, double m_y, double m_th
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    /* Set first circular path parameters just once. */
    static inline void setCircularPathParameters(TRVFtrajectory *TRVFtraj);
    
    /* Set circular parameters for second circular path just once. 
    Arguments:
      lineangle: entering ray angle (in rad and in relation to global x-axis) of the central angle region angle.
      TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj: parameters of the TRVF trajectory. */
    static inline void setCircularPathParameters2(int linenumber, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj);
    
    /* Cicular path vector field constructor. Modifies the parameters fx and fy. 
    Returns a float value: <= 0 and >= 1 indicates the robot already reached the final waypoint. */
    static double circularPathFolowing(double &fx, double &fy, TRVFtrajectory *TRVFtraj, double m_x, double m_y, double m_th
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    /* Follow the line-circle-line vector field. Modifies fx, fy, m_state, finished, currentWaypoint,  pos, theWorld, numIterationsReachGoal, numIterations and reachingTargetTime.
    Reference: Vector field path following for small unmanned air vehicles.
    Conference Paper in Proceedings of the American Control Conference · July 2006
    DOI: 10.1109/ACC.2006.1657648 */
    static void coolPathFollowing(double& fx, double& fy, States &m_state, bool &finished, unsigned int & currentWaypoint, World* theWorld, unsigned int &numIterationsReachGoal, unsigned int &numIterations, Stg::usec_t &reachingTargetTime, TRVFparameters* TRVFparam, TRVFtrajectory *TRVFtraj, double m_x, double m_y, double m_th, double destinationX, double destinationY, unsigned int m_id 
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    //Alter the values of fx and fy, adding repulsion force by target region.
    static void targetRegionRepulsionForce(double& fx, double& fy,  double m_x, double m_y
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    // Reads a config file and calculate the static parameters.
    static void readConfigFile(ConfigFile *cf);
    
    //For checking if configuration was done yet.
    static bool alreadyConfigurated;
    
    static TRVFparameters TRVFparam;
    
    TRVFtrajectory TRVFtraj;
};

#endif
