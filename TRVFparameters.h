/*Implements the static common parameters of TRVF algorithm*/
#ifndef _TRVFPARAMETERS_H_
#define _TRVFPARAMETERS_H_

#include <math.h>
#include "commonConfig.h"

class TRVFparameters{
  public:
    //number of lanes
    unsigned int K_path;
    
    //trajectory parameters
    double alpha, r, dcurve, corridorLength, powtauk;
    
    //calculate parameters given K_path, the radius of the target circular area, the distance between the robots d and the algorithm range D
    void calculateParameters(double waypointDist, double d, double D);
};

#endif
