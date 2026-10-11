#include <stage.hh>
#include <iostream>
#include <string.h>
#include "commonConfig.h"
#include "Robot.h"
#include "TRVF.h"
#include "NoCoord.h"
#include "SQF.h"
#include "FollowNeighbour.h"
#include "RLRobot.h"


//Reads a configuration file and calculate the static parameters only once. After this procedure, cf is updated if was not initialised.
void readConfigFile(string const& confFileName);

// Delete the configuration file data. 
void deleteConfigFile();

//Function used on Stage simulation. It specifies 
//what the robot will do while walking
int PositionUpdate(Model *pos, Robot *robot);

// Implements a factory of robots based on the chosen algorithm. 
Robot* fabricateRobot();
