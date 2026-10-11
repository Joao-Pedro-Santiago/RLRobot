#include "TRVFRegionsVisualizer.h"

//Reads a config file and intializes the private members.
void TRVFRegionsVisualizer::readConfigFile(ConfigFile *cf){
  try{
    d = atof(cf->valueOf("d").c_str());
    K = atoi(cf->valueOf("K").c_str());
  }
  catch(string s) {
    cout << "TRVFdraw.so: Configuration file is incorret: " << s << endl;
    exit(1);
  }
}

//Draw target region and corridors
void TRVFRegionsVisualizer::drawRegions(){
  double angle;
  int i;
  for (i=0, angle=0.0; i < K; i++, angle+=2*PI/K){
    double ca = cos(angle);
    double sa = sin(angle);
    double X1 = waypoints[0][0] + dcurve*ca;
    double Y1 = waypoints[0][1] + dcurve*sa;
    double X2 = waypoints[0][0] + (dcurve + corridorLength)*ca;
    double Y2 = waypoints[0][1] + (dcurve + corridorLength)*sa;
    double d2sa = (d/2)*sa;
    double d2ca = (d/2)*ca;
    /*
    Corridor variables depicting:
    
     (X1-d2sa,Y1+d2ca)          (X2-d2sa,Y2+d2ca)
              __________________________
             |                          |
     (X1,Y1) --------------------------- (X2,Y2)
             |__________________________|
     (X1+d2sa,Y1-d2ca)          (X2+d2sa,Y2-d2ca)
    */
    glBegin(GL_LINES);
      glVertex2f(X1, Y1);
      glVertex2f(X2, Y2);
    glEnd();
    glBegin(GL_LINE_LOOP);
      //d cos(90º+a) = - d sin(a) and d sin(90º+a)  = d cos(a).
      glVertex2f(X1 - d2sa, Y1 + d2ca);
      glVertex2f(X2 - d2sa, Y2 + d2ca);
      //d cos(-90º+a) = d sin(a) and d sin(-90º+a)  = - d cos(a).
      glVertex2f(X2 + d2sa, Y2 - d2ca);
      glVertex2f(X1 + d2sa, Y1 - d2ca);
    glEnd();
          
    double cb = cos(angle - (2*PI/K));
    double sb = sin(angle - (2*PI/K));
    double d2sb = (d/2)*sb;
    double d2cb = (d/2)*cb;
    double X4 = waypoints[0][0] + dcurve*cb - d2sb, 
           Y4 = waypoints[0][1] + dcurve*sb + d2cb; //previous angle, pointing up
    double cx = X4 - r*sb, cy = Y4 + r*cb;
    //semicircle for turning
    glBegin(GL_LINE_STRIP);
      double ang, angB =  PI/2 + angle, angE = 3*PI/2 + angle - 2*PI/K;
      for (ang=angB; ang <= angE; ang+=(angE - angB)/53)
      {
         glVertex2f(cos(ang)*r + cx, sin(ang)*r + cy);
      }
    glEnd();
  }
  drawTargetArea();
  drawEffectArea();
}

TRVFRegionsVisualizer::TRVFRegionsVisualizer(ConfigFile *cf):TargetAreaVisualizer(cf){
  readConfigFile(cf);
  alpha = 2*PI/K;
  r = (waypointDist * sin(alpha / 2.) - d/2.) / (1 - sin(alpha / 2.));
  dcurve = sqrt(pow(waypointDist + r,2) - pow(d/2. + r,2));
  corridorLength = D - dcurve;
}

void TRVFRegionsVisualizer::Visualize( Model* mod, Camera* cam ){
  glColor3f(0,0,0);
  drawRegions();
}

TRVFRegionsVisualizer* rect;

extern "C" int Init(Model *mod, CtrlArgs *args){
  vector<string> tokens;
  Tokenize(args->worldfile, tokens);
  rect = new TRVFRegionsVisualizer(new ConfigFile(tokens[1]));
  mod->Subscribe();
  mod->AddVisualizer( rect, true );
  return 0;
}
