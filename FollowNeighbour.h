/*
  This file implements robots that follows the first neighbour they detect.
*/
#ifndef _FOLLOWNEIGHBOUR_H_
#define _FOLLOWNEIGHBOUR_H_

#include "Robot.h"
#include "TRVF.h"
#include "SQF.h"

class FollowNeighbour : public Robot
{
  public:
    /*Constructor arguments:
       conf: file name with experiments variables.
    */
    FollowNeighbour(ConfigFile* confFile);
    
    //Initialize robot data for no coordination algorithm.
    void init(int id);
    
    //Implements the main loop of robot. 
    //Also contain robot controller and 
    //probabilistic finite state machine codes
    void mainLoop();
  
  private:
    //Returnsthe state of the experiment ending depending on the alternative algorithm.
    inline States algorithmFinalState();
    
    //Return the truth value of the condition to finish the experiment depending on the alternative algorithm
    inline bool algorithmFinishingCondition();
  
    // Check if there is any robot in neighbour and follow the one next to its current direction.
    // If none is found, the robot executes the alternative algorithm.
    void checkNeighbourhoodAndFollow();
    
    //Reads a configuration file and calculate the static parameters only once. After this procedure, cf is updated if was not initialised.
    void readConfigFile(ConfigFile *cf);
    
    //Convert an algorithm in C string to Algorithms code.
    static Algorithms strToAlgorithm(const char* str);
    
    // For checking if configuration was done yet.
    static bool alreadyConfigurated;
    
    // Minimum value for the dot product between the target direction and the laser beam direction which detected a neighbour, that is, cos(chosen angle) * maximum attractive force modulus.
    static double minDotProductToFollow;
    
    static Algorithms alternativeAlgorithm;
    
    static TRVFparameters TRVFparam;
    
    TRVFtrajectory TRVFtraj;
};

#endif
