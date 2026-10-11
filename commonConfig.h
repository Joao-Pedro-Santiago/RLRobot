#ifndef _COMMONCONFIG_H_
#define _COMMONCONFIG_H_

//Waypoints chosen randomly
const int waypoints[3][2] = { {100,100}, {-999999, 100}, {999999,100}/*, {0,-999999}, {0,999999}*/ };
#define NUMBER_OF_WAYPOINTS 2 //number of waypoints used in all tests
//~ #define DEBUG_FORCES //If defined, it allows compilation of force visualization codes

//how much is next to the target region to consider that the robot touched it. Useful because of the precision of double floating point numbers.
const double epsilon = 0.00001;

//how much is next to the waypoint at the end of the first line path to consider that the robot touched it.
const double epsilonLineFollowing = 0.25;

//number of iterations to wait for the last robot changes its colour. If it is 1, it changes immediately. This is used for generate videos or plotting images, so that the last robot will be black. Use 99 when do that. 
const int ITERATION_FOR_CHANGING_COLOR = 1;

const double Ka = 2.5; //constant for robot controller, attraction to target
const double Kr = 3; //constant for robot controller, m/s^2/rad acceleration in rotation
const double Kl = 1; //constant for robot controller. Factor of the input acceleration (should be set to 1 by default)
const double Kdp = 10; // Weight of the damping force
const double Kobs = 0.5;  //constant for repulsive forces
const double maxForce = 2.5; //Maximum modulo for resultant force, used on robot controller
const float Kexpoent1 = 1.1; //constant for the line trajectory vector field generator in the required power operations
const float Kexpoent2 = 1.1; //constant for the curved trajectory vector field generator in the required power operations

const unsigned long long FINISH_TIME = 100000; //Extra time to finish simulation;
const unsigned long long maxTestTime = 600l*60*1000000; // 600 min converted to microseconds

#define PI M_PI
#endif
