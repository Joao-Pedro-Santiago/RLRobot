#include "SQFRegionsVisualizer.h"

//Draw target region and corridor
void SQFRegionsVisualizer::drawRegions(){
  glBegin(GL_LINES);
    glVertex2f(waypoints[0][0]-waypointDist,waypoints[0][1]);
    glVertex2f(waypoints[0][0]-waypointDist,waypoints[0][1]+corridorLength);
  glEnd();
  glBegin(GL_LINES);
    glVertex2f(waypoints[0][0]+waypointDist,waypoints[0][1]);
    glVertex2f(waypoints[0][0]+waypointDist,waypoints[0][1]+corridorLength);
  glEnd();
  drawTargetArea();
  drawEffectArea();
}

SQFRegionsVisualizer::SQFRegionsVisualizer(ConfigFile *cf):TargetAreaVisualizer(cf){
  corridorLength = sqrt(D*D - waypointDist*waypointDist);
}

void SQFRegionsVisualizer::Visualize( Model* mod, Camera* cam ){
  glColor3f(0,0,0);
  drawRegions();
}

SQFRegionsVisualizer* rect;

extern "C" int Init(Model *mod, CtrlArgs *args){
  vector<string> tokens;
  Tokenize(args->worldfile, tokens);
  rect = new SQFRegionsVisualizer(new ConfigFile(tokens[1]));
  mod->Subscribe();
  mod->AddVisualizer( rect, true );
  return 0;
}
