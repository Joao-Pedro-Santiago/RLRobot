/*
  This file implements a shared library using OpenGl functions to draw lines and circles on Stage for TRVF algorithm.
*/
#ifndef _SQFREGIONSVISUALIZER_H_
#define _SQFREGIONSVISUALIZER_H_

#include "TargetAreaVisualizer.h"

class SQFRegionsVisualizer: public TargetAreaVisualizer{
  private:
    double corridorLength;
  
    //Draw target region and corridor
    void drawRegions();
  
  public:
    SQFRegionsVisualizer(ConfigFile *cf);
  
    void Visualize( Model* mod, Camera* cam );
};

#endif
