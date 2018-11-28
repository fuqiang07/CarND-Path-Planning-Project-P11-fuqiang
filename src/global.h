#ifndef GLOBAL_H
#define GLOBAL_H

#include <vector>
#include <string>
#include <math.h>

using namespace std;
/*
 * **************************************************************** *
 *        Declare Global Constants and Variables
 * **************************************************************** *
*/
extern const double GLOBAL_MAX_DOUBLE;		//maximum double value

extern const int GLOBAL_NUM_POINTS; 		//the number of points in the trajectory sent to simulator
extern const double GLOBAL_TS; 				//sampling time in this simulation
extern const double GLOBAL_LANE_WIDTH;		//lane width

extern const string GLOBAL_MAP_FILE;		//waypoint map file 
extern const double GLOBAL_MAX_S;			//max s value in the map
extern const double GLOBAL_CENTER_X;		//center point of the track
extern const double GLOBAL_CENTER_Y;	
//extern const int GLOBAL_NUM_LANES; 			//the number of lane in the simulation
const int GLOBAL_NUM_LANES = 3; 			//the number of lane in the simulation

extern const double GLOBAL_MAX_SPEED_MPH; 	//maximum speed in mph
extern const double GLOBAL_MAX_SPEED; 		//maximum speed in ms
extern const double GLOBAL_MAX_ACCEL; 		//maximum acc in ms2
extern const double GLOBAL_MAX_JERK; 		//maximum jerk in ms3
extern const double GLOBAL_MAX_SPEED_INC; 	//speed increment in ms2
extern const double GLOBAL_MAX_SPEED_INC_MPH;

extern const double GLOBAL_SD_LC;			//safety distance for lane change		
extern const double GLOBAL_SD; 				//safety distance for current lane keeping

extern const double GLOBAL_FIELD_OF_VIEW; 	// Field Of View

extern const int GLOBAL_PREV_PATH_XY_REUSED; //simulation latency 

extern const bool GLOBAL_TRAJECTORY_JMT;

// some extra margin for safety: safety box around the car
extern const double GLOBAL_CAR_SAFETY_W; // meters
extern const double GLOBAL_CAR_SAFETY_L; // meters
extern const int GLOBAL_MAX_COLLISION_STEP;

// cf cost.cpp: weighted cost function from most critical to less critical one
extern const int GLOBAL_COST_FEASIBILITY; // vs collisions, vs vehicle capabilities
extern const int GLOBAL_COST_SAFETY; // vs buffer distance, vs visibility or curvature
extern const int GLOBAL_COST_LEGALITY; // vs speed limits
extern const int GLOBAL_COST_COMFORT; // vs jerk
extern const int GLOBAL_COST_EFFICIENCY; // vs target lane, target speed and time to goal

//to store data related to car
struct CarData {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double speed_target;
  int    lane;
  bool   emergency;
  CarData (double X=0, double Y=0, double S=0, double D=0, double YAW=0, 
           double V=0, double VF=0, double L=0, bool E=false) : x(X), y(Y), s(S), yaw(YAW), 
           speed(V), speed_target(VF), lane(L), emergency(E) {}
};

//to store coordinate values 
struct Coord {
  double x;
  double y;
};


/*
 * **************************************************************** *
 *        Declare Helper Functions
 * **************************************************************** *
*/
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);
// For converting back and forth between mph and ms.
double mph2ms(double vel_mph);
double ms2mph(double vel_ms);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Get lane ID based on the d value of the car
unsigned int get_lane(double car_d);

// d coord for "left lane" of a lane
double get_dleft(int lane);
// d coord for "right lane" of a lane
double get_dright(int lane);
// d coord for "center lane" of a lane
double get_dcenter(int lane);

#endif