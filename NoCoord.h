/*
  This file implements robots only following attractive and repulsive forces.
*/
#ifndef _NOCOORD_H_
#define _NOCOORD_H_

#include "Robot.h"

class NoCoord : public Robot
{
  public:
    /*Constructor arguments:
       conf: file name with experiments variables.
    */
    NoCoord(ConfigFile* confFile);
    
    //Initialize robot data for no coordination algorithm.
    void init(int id);
    
    //Implements the main loop of robot. 
    //Also contain robot controller and 
    //probabilistic finite state machine codes
    void mainLoop();
};

#endif
