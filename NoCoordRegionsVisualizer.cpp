#include "NoCoordRegionsVisualizer.h"

//Draw target region and corridor
void NoCoordRegionsVisualizer::drawRegions(){
  drawTargetArea();
  drawEffectArea();
}

NoCoordRegionsVisualizer::NoCoordRegionsVisualizer(ConfigFile *cf):TargetAreaVisualizer(cf){}

void NoCoordRegionsVisualizer::Visualize( Model* mod, Camera* cam ){
  glColor3f(0,0,0);
  drawRegions();
}

NoCoordRegionsVisualizer* rect;

extern "C" int Init(Model *mod, CtrlArgs *args){
  vector<string> tokens;
  Tokenize(args->worldfile, tokens);
  rect = new NoCoordRegionsVisualizer(new ConfigFile(tokens[1]));
  mod->Subscribe();
  mod->AddVisualizer( rect, true );
  return 0;
}

