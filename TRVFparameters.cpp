#include "TRVFparameters.h"

//calculate parameters given K_path, the radius of the target circular area, the distance between the robots d and the algorithm range D
void TRVFparameters::calculateParameters(double waypointDist, double d, double D){
  alpha = 2*PI/K_path;
  r = (waypointDist * sin(alpha / 2.) - d/2.) / (1 - sin(alpha / 2.));
  dcurve = sqrt(pow(waypointDist + r,2) - pow(d/2. + r,2));
  corridorLength = D - dcurve;
  powtauk = pow(d/5,Kexpoent1);
}
