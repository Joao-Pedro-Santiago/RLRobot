/*
  Creates automatically scenarios for stage.
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include "commonConfig.h"
#include "ConfigFile.h"

#include <iomanip>

using namespace std;

typedef struct coord
{
      double x;
      double y;
      double theta; //in radians
} coord;

bool intersect(coord newCoord, vector<coord> &history, double mindist)
{
   for(unsigned int i = 0; i < history.size(); i++)
   {
      if (sqrt(pow(newCoord.x - history.at(i).x,2) + pow(newCoord.y - history.at(i).y,2)) < mindist)
        return true;
   }
   return false;
}

void rand(coord &newCoord, vector<coord> &history, double minRadius, double maxRadius, double mindist)
{
   double r;
   double pho;
   double theta;

   while(true)
   {
      r = (   (double)rand() / ((double)(RAND_MAX)+(double)(1)) );
      pho = minRadius + r*(maxRadius - minRadius); 
      r = (   (double)rand() / ((double)(RAND_MAX)+(double)(1)) );
      theta = 0 + r*2*PI;

      newCoord.x = pho*cos(theta);
      newCoord.y = pho*sin(theta);
      newCoord.theta = atan2(0 - newCoord.y, 0 - newCoord.x); 

      newCoord.x = waypoints[0][0] + newCoord.x;
      newCoord.y = waypoints[0][1] + newCoord.y;

      if (!intersect(newCoord, history, mindist))
        break;
   }
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    cerr << endl
         <<"Invalid parameters" << endl;
    cerr << "Use: " << endl;
    cerr << "createScenario  <configuration file> [video]" << endl;
    exit(1);
  }
  string automatic;
  string confFile(argv[1]);
  ConfigFile cf(confFile);
  unsigned int n; // number of robots knowing the algorithm
  unsigned int m; // number of ad hoc robots
  double D;
  bool isHolonomic;
  int seed;
  char * alg_name = NULL;
  
  try{
    n = atoi(cf.valueOf("n").c_str());
    m = atoi(cf.valueOf("m").c_str());
    D = atof(cf.valueOf("D").c_str());
    automatic = cf.valueOf("scenario");
    isHolonomic = (atoi(cf.valueOf("holo").c_str()) != 0);
    alg_name = (char*) cf.valueOf("algorithm").c_str();
  }
  catch(string str){
    cerr << endl << "Configuration file is incorret: " << str << endl;
    exit(1);
  }
  try{
    seed = atoi(cf.valueOf("seed").c_str());
  }
  catch(string str){
    seed = -1;
  }
  
  const char* allowedAlgorithms[] = {"TRVF","NoCoord","SQF"};
  size_t aAsize = sizeof(allowedAlgorithms)/sizeof(char *);
  bool alg_name_is_allowed = false;
  for (unsigned int i = 0; i < aAsize; i++){
    if (strcmp(allowedAlgorithms[i],alg_name) == 0){
      alg_name_is_allowed = true;
      break;
    }
  }
  if (!alg_name_is_allowed){
    cerr << "Invalid algorithm name." << endl;
    cerr << "Use any of the following: " << endl;
    for (unsigned int i = 0; i < aAsize; i++){
      cout << allowedAlgorithms[i] << " ";
    }
    cout << endl;
    exit(1);
  }
  
  vector<coord> history;
  coord tmp;
  int numRobot = 0;

  ofstream output((automatic + ".world").c_str());
  output << "# defines 'map' object used for floorplans" << endl
    << "include \"map.inc\"" << endl << endl

    << "# defines sick laser" << endl
    << "include \"sick.inc\"" << endl << endl
    << "# defines Pioneer-like robots" << endl
    << "include \"pioneer.inc\"" << endl << endl
    << "# set the resolution of the underlying raytrace model in meters" << endl
    << "resolution 0.1" << endl
    << "" << endl
    << "speedup -1" << endl
    << "" << endl
    << "# configure the GUI window" << endl
    << "window" << endl
    << "( " << endl
    << "  size [ 591.000 638.000 ] " << endl
    << "  center [ " << waypoints[0][0] <<  " " << waypoints[0][1] << " ] " << endl
    << "  show_data 1" << endl;

  if (argc >= 3){
    if (strcmp(argv[2],"video") == 0){
      output << "  screenshots 1" << endl;
    }
  }

  output << ")" << endl
    << "define robot pioneer2dx" << endl
    << "(" << endl
    << "  sicklaser ()" << endl
    << "  size [0.44 0.44 0.44]" << endl
    << "  localization \"gps\"" << endl
    << "  localization_origin [ 0 0 0 0 ]" << endl;
  if (isHolonomic){
    output << "  drive \"omni\"" << endl;
  }
  output  << ")" << endl 
    << endl
    << "model" << endl
    << "(" << endl
    << "  name \"toDraw\""<< endl
    << "  size [0.001 0.001 0.001]"<< endl
    << "  pose [0 0 0 0]"<< endl
    << "  ctrl \"" << alg_name << "draw.so " << confFile << " \""<< endl
    << "  obstacle_return 0" << endl
    << ")"<< endl << endl;
  if (seed >= 0){
    srand(seed);
  }
  else{
    srand(time(NULL));
  }
  for(unsigned int i = 0; i < n+m; i++)
  {
    rand(tmp, history, D, D + 8.,1);  
    history.push_back(tmp);
    output << "robot" << endl
      << "(" << endl
      << "  name \"robot" << numRobot << "\"" << endl
      << "  color \"red\"" << endl
      << setprecision(17)
      << "  pose [" << tmp.x << " " << tmp.y << " 0 " << ((isHolonomic)? 0 : (180/PI)*tmp.theta) << " ]" << endl
      << "  ctrl \"coordination.so " << confFile << " " << numRobot <<" " << "\"" << endl
      << ")" << endl << endl;
    
    numRobot++;
  }
   
  output.close();
  return 0;
}
