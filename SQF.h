/*
  This file implements the SQF algorithm.
*/
#ifndef _SQF_H_
#define _SQF_H_

#include "Robot.h"

class SQF : public Robot
{
  public:
    /*Constructor arguments:
       conf: file name with experiments variables.
    */
    SQF(ConfigFile* confFile);
    
    //Initialize robot data for SQF algorithm.
    void init(int id);
    
    //Implements the main loop of robot. 
    //Also contain robot controller and 
    //probabilistic finite state machine codes
    void mainLoop();

    // Calculate the TRVF force depending on the robot's state
    static void calculateStateDependentForce(double& fx, double& fy, States &m_state, ModelPosition* pos, double m_x, double m_y, double m_th, double destinationX, double destinationY
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    //Alters the values of fx and fy, adding repulsion force. Also, it updates the mean and variance of distances between robots sensed by laser ranges.
    static void obstaclesRepulsionForces(double &fx, double &fy, double* mean_distance, double* var_distance, unsigned long &n_distances, ModelRanger* laser, bool finished, States m_state, double m_x, double m_y, double m_th, double destinationX, double destinationY
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
  protected:

    // Calculate rotation force field for entering 
    static void calculateEnteringRotationForce(double& fx, double& fy, double m_x, double m_y, double destinationX, double destinationY
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
    
    // Calculate rotation force field for leaving 
    static void calculateLeavingRotationForce(double& fx, double& fy, double m_x, double m_y, double destinationX, double destinationY, double D
    #ifdef DEBUG_FORCES
      , ForceVisualizer& fv
    #endif
    );
};

#endif
