#include "init.h"

using namespace std;

//Pointer to the only configuration file used.
ConfigFile* cf = NULL;
char * chosenAlgorithm = NULL;
unsigned int nKnowing;

//Reads a configuration file and calculate the static parameters only once. After this procedure, cf is updated if was not initialised.
void readConfigFile(string const& confFileName)
{
  if (cf != NULL) return;
  cf = new ConfigFile(confFileName);
  try{
    chosenAlgorithm = (char*) cf->valueOf("algorithm").c_str();
    nKnowing        = atoi(cf->valueOf("n").c_str());
  }
  catch (string s) {
    cout << endl << "init.o: Configuration file is incorret: " << s << endl;
    exit(1);
  }
}

// Delete the configuration file data. 
void deleteConfigFile()
{
  if (cf != NULL){
    delete cf;
    cf = NULL;
  }
}

//Function used on Stage simulation. It specifies 
//what the robot will do while walking
//See Robot.h for signature
int PositionUpdate(Model *pos, Robot *robot)
{
  robot->mainLoop();
  return 0;
}

// Implements a factory of robots based on the chosen algorithm. 
Robot* generateRobot()
{
  if (strcmp(chosenAlgorithm,"TRVF") == 0){
    return new TRVF(cf);
  }
  else if (strcmp(chosenAlgorithm,"NoCoord") == 0){
    return new NoCoord(cf);
  }
  else if (strcmp(chosenAlgorithm,"SQF") == 0){
    return new SQF(cf);
  }
  else{
    cout << "Algorithm " << chosenAlgorithm << " not implemented." << endl;
    cout << "Available algorithms: " << endl
         << "TRVF NoCoord SQF." << endl;
    exit(1);
  }
  return NULL;
}

//Pointer to a new robot.
//Every call of this library will create a new robot
//using this pointer.
Robot* robot;

extern "C" int Init(Model *mod, CtrlArgs *args)
{
  vector<string> tokens;
  Tokenize(args->worldfile, tokens); 
  if (tokens.size() < 3){
    cout << endl;
    cout << "Wrong number of arguments." << endl;
    cout << "Usage:" << endl
         << "  coordination.so <config file> <robot id>" << endl;
    exit(1);
  }
  readConfigFile(tokens[1]);
  unsigned int myId = atoi(tokens[2].c_str());
  robot = (myId < nKnowing)? generateRobot() : new RLRobot(cf);
  ModelPosition *pmod = (ModelPosition*) mod;
  robot->pos = pmod;
  robot->theWorld = pmod->GetWorld(); //ゴ ゴ ゴ ゴ
  robot->pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
  robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );
  
  robot->laser->Subscribe(); // starts the laser updates
  robot->pos->Subscribe(); // starts the position updates
  
  robot->init(atoi(tokens[2].c_str()));
  #ifdef DEBUG_FORCES
  robot->pos->AddVisualizer( &robot->fv, true );
  #endif
  deleteConfigFile();
  return 0;
}
