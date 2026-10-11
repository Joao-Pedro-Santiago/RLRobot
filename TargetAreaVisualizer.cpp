#include "TargetAreaVisualizer.h"

// circle of target size
void TargetAreaVisualizer::drawTargetArea(){
  glBegin(GL_LINE_LOOP);
    for (double angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
    {
       glVertex2f(cos(angle)*waypointDist + waypoints[0][0],
                  sin(angle)*waypointDist + waypoints[0][1]);
    }
  glEnd();
}

// draw the circle of the algorithm effect.
void TargetAreaVisualizer::drawEffectArea(){
  glBegin(GL_LINE_LOOP);
    for (double angle=0.0; angle < 2*PI; angle+=10*(PI/180.0))
    {
       glVertex2f(cos(angle)*D + waypoints[0][0],
                  sin(angle)*D + waypoints[0][1]);
    }
  glEnd();
}
    
//Reads a config file and intializes the private members.
void TargetAreaVisualizer::readConfigFile(ConfigFile *cf){
  try{
    waypointDist = atof(cf->valueOf("s").c_str());
    D = atof(cf->valueOf("D").c_str());
  }
  catch(string s) {
    cout << "Configuration file is incorret: " << s << endl;
    exit(1);
  }
}
  
TargetAreaVisualizer::TargetAreaVisualizer(ConfigFile *cf):Visualizer("reg","vis_reg"){
  readConfigFile(cf);
}
