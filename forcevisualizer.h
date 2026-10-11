/*
  This class implements the visualization of all forces aplied on a single robot
  Used for debug
*/
#ifndef _FORCEVISUALIZER_H_
#define _FORCEVISUALIZER_H_

#include <stage.hh>

#define SHOW_REPULSIVE //If defined, it allows to show repulsive forces between robots and obstacles 
#define SHOW_TARGET_REPULSIVE //If defined, it allows to show target region repulsive forces between robots and obstacles 
#define SHOW_ATTRACTIVE //If define, it allows to show attractive force to goal
#define SHOW_RESULTANT //If defined, it allow to show the resultant force on robot

using namespace Stg;

//defines a point 
typedef struct _Point_{
  double x,y;
} Point;

//implement visualization of vectors
class ForceVisualizer : public Visualizer{
  public:
    //this function must to be used every time the robot changes your position 
    void setPosition(double m_x, double m_y, double m_th);

    //sets repulsive force value
    void setRepulsiveForces(double fxrep, double fyrep);
   
    //sets repulsive force value by target region
    void setTargetRepulsiveForces(double fxtrep, double fytrep);
   
    //sets attractive force value
    void setAttractiveForces(double fxatt, double fyatt);

    //set the resultant force value
    void setResultantForces(double fx, double fy);
    
    //This constructor. Initializes forces with zeros.
    ForceVisualizer();
  private:

    //repulsive force values
    double fxrep,fyrep;
    
    //repulsive target force values
    double fxtrep,fytrep;

    //attractive force values
    double fxatt,fyatt;

    //resultant force values
    double fxR, fyR;

    //robot's pose 
    double m_x,m_y,m_th;

    //Transformation used to plot forces over robot view point
    Point T(double x, double y);

    //Method to Visualize forces. Inherited from Stage API.
    void Visualize( Model* mod, Camera* cam );

    //Draw a vertex relative to robot's position
    inline void _glVertex2f(double x, double y);
};

#endif
