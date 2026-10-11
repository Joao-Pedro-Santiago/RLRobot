/*
  This file implements a shared library using OpenGl functions to draw lines and circles on Stage for TRVF algorithm.
*/
#ifndef _TRVFREGIONSVISUALIZER_H_
#define _TRVFREGIONSVISUALIZER_H_

#include "TargetAreaVisualizer.h"

class TRVFRegionsVisualizer: public TargetAreaVisualizer{
  private:
    double d; //minimum distance between the robots
    double r; //r adjusted
    double dcurve; //distance from the lane to the beginning of the curve.
    double corridorLength; //size of the lanes, before the curved path.
    double alpha; //central area region angle
    int K; //number of lanes

    //Reads a config file and intializes the private members.
    void readConfigFile(ConfigFile *cf);
    
    //Draw target region and corridors
    void drawRegions();
  
  public:
    TRVFRegionsVisualizer(ConfigFile *cf);
  
    void Visualize( Model* mod, Camera* cam );
};

#endif
