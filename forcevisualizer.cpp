#include "forcevisualizer.h"

//this function must to be used every time the robot changes your position 
void ForceVisualizer::setPosition(double m_x, double m_y, double m_th){
  this->m_x = m_x;
  this->m_y = m_y; 
  this->m_th = m_th; 
}

//sets repulsive force value
void ForceVisualizer::setRepulsiveForces(double fxrep, double fyrep){
  this->fxrep = fxrep; 
  this->fyrep = fyrep;  
}

//sets repulsive force value by target region
void ForceVisualizer::setTargetRepulsiveForces(double fxtrep, double fytrep){
  this->fxtrep = fxtrep; 
  this->fytrep = fytrep;
}

//sets attractive force value
void ForceVisualizer::setAttractiveForces(double fxatt, double fyatt){
  this->fxatt = fxatt;
  this->fyatt = fyatt;
}

//resultant force values
void ForceVisualizer::setResultantForces(double fxR, double fyR)
{
  this->fxR = fxR;
  this->fyR = fyR;
}

//This constructor. Initializes forces with zeros.
ForceVisualizer::ForceVisualizer():Visualizer("rects","vis_rects"){
  fxR = fyR = 
  fxtrep = fytrep = 
  fxrep = fyrep =
  fxatt = fyatt = 0;
}

//Transformation used to plot forces over robot view point
Point ForceVisualizer::T(double x, double y){
  Point p;
  p.x = x*cos(-m_th) - y*sin(-m_th);
  p.y = x*sin(-m_th) + y*cos(-m_th);
  return p;
}

//Draw a vertex relative to robot's position
inline void ForceVisualizer::_glVertex2f(double x, double y){
  Point p = T(x,y);
  glVertex2f(p.x,p.y);
}

//Method to Visualize forces. Inherited from Stage API.
void ForceVisualizer::Visualize( Model* mod, Camera* cam ){
  glBegin( GL_LINES );
    #ifdef SHOW_REPULSIVE
    glColor3f(0,0,1); //Blue
    _glVertex2f(0,0);
    _glVertex2f(fxrep,fyrep);
    #endif
    #ifdef SHOW_TARGET_REPULSIVE
    glColor3f(0,1,1); //Cyan
    _glVertex2f(0,0);
    _glVertex2f(fxtrep,fytrep);
    #endif
    #ifdef SHOW_ATTRACTIVE
    glColor3f(1,0,0); //Red
    _glVertex2f(0,0);
    _glVertex2f(fxatt,fyatt);
    #endif
    #ifdef SHOW_LINE_REPULSIVE
    glColor3f(0,1,0); //Green
    _glVertex2f(0,0);
    _glVertex2f(fxlin,fylin);
    #endif
    #ifdef SHOW_RESULTANT
    glColor3f(0,0,0); //Black
    _glVertex2f(0,0);
    _glVertex2f(fxR,fyR);
    #endif
  glEnd();  
}
