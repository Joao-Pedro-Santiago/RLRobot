#ifndef _COMMONDEFS_H_
#define _COMMONDEFS_H_


//Define the codes to each state of prob. finite state machine
typedef enum _States {
  GOING, 
  GOING_OUT,
  WAIT, 
  ENTERING_OUTSIDE,
  ENTERING_LINE1,
  ENTERING_CURVE,
  LEAVING_CURVE,
  LEAVING_LINE2,
  FOLLOW_NEIGHBOUR,
  INVALID_STATE
} States;

//Define the codes to each algorithm
typedef enum _Algorithms{
  TRVF_ALGORITHM,
  SQF_ALGORITHM,
  NOCOORD_ALGORITHM
} Algorithms;

//define the colors used in simulation
//Colors                                  R        G          B
#define GOING_COLOR             Color(        1,        0,        0) //red
#define WAIT_COLOR              Color(        0,        1,        0) //green
#define ENTERING_OUTSIDE_COLOR  Color(        0,        1,        1) //cyan
#define ENTERING_LINE1_COLOR    Color(        0,        0,        1) //blue
#define ENTERING_CURVE_COLOR    Color(        1,        0,        1) //magenta
#define LEAVING_CURVE_COLOR     Color(        1,        1,        0) //yellow
#define LEAVING_LINE2_COLOR     Color(        1,165./255.,        0) //orange
#define END_COLOR               Color(        0,        0,        0) //black
#define FOLLOW_NEIGHBOUR_COLOR  Color( 66./255., 28./255., 82./255.) //purple
#define GOING_ADHOC_COLOR       Color(      0.7,      0.7,      0.7) //light grey
#define END_ADHOC_COLOR         Color(      0.2,      0.2,      0.2) //dark grey
#define GOING_OUT_COLOR         Color(        1,      0.7,        0) //dark yellow
#define RLGOING_COLOR           Color(      0.6,      0.0,      1.0) //violet       
#define RLGOING_OUT_COLOR       Color(     0.29,        0,     0.51) // anil blue
#define RLGOING_END_COLOR       Color(      0.3,      0.0,      0.5) //dark violet


#endif
