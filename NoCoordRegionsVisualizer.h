/*
  This file implement a shared library using OpenGl functions to draw circles on Stage for NoCoord algorithm.
*/
#ifndef _NOCOORDREGIONSVISUALIZER_H_
#define _NOCOORDREGIONSVISUALIZER_H_

#include "TargetAreaVisualizer.h"

class NoCoordRegionsVisualizer: public TargetAreaVisualizer{
  private:
    //Draw target region and corridor
    void drawRegions();
  
  public:
    NoCoordRegionsVisualizer(ConfigFile *cf);
  
    void Visualize( Model* mod, Camera* cam );
};

#endif
