/*
  This file implements a class to draw lines or circles on Stage for various algorithms.
*/ 
#ifndef _TARGETAREAVISUALIZER_H_
#define _TARGETAREAVISUALIZER_H_

#include <stage.hh>
#include "commonConfig.h"
#include "ConfigFile.h"
#include "util.h"

using namespace Stg;
using namespace std;

#define PI M_PI

//Define a point
struct _Ponto{
  double px;
  double py;
};

typedef struct _Ponto Ponto;

class TargetAreaVisualizer: public Visualizer{
  protected:
    double D; //maximum distance from the target to algorithm takes effect.
    double waypointDist; //radius of target area.

    // circle of target size
    void drawTargetArea();
    
    // draw the circle of the algorithm effect.
    void drawEffectArea();
    
    //Reads a config file and intializes the private members.
    void readConfigFile(ConfigFile *cf);
  
  public:
    TargetAreaVisualizer(ConfigFile *cf);
    
    virtual void Visualize( Model* mod, Camera* cam ) = 0;
};

#endif
