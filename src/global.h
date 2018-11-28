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


extern static const double GLOBAL_LANE_WIDTH;	//lane width
extern static const string GLOBAL_MAP_FILE;	//waypoint map file 
extern static const double GLOBAL_MAX_S;		//max s value in the map
extern static const double GLOBAL_CENTER_X;	//center point of the track
extern static const double GLOBAL_CENTER_Y;	



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
unsigned int getLaneID(double car_d);



/* // Computationnal defines
#define INF 1e10

enum {
  ID = 0, // 0
  X  = 1, // 1
  Y  = 2, // 2
  VX = 3, // 3
  VY = 4, // 4
  S  = 5, // 5
  D  = 6, // 6
  SIZE    // 7
};

struct Coord {
  double x;
  double y;
};

struct Frenet {
  double s;
  double d;
};


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

typedef std::vector<double > t_coord;
typedef std::vector<t_coord> t_traj;
typedef std::vector<t_traj > t_trajSet;



// d coord for "left lane" of a lane
double get_dleft(int lane);
// d coord for "right lane" of a lane
double get_dright(int lane);
// d coord for "center lane" of a lane
double get_dcenter(int lane);

int get_lane(double d);

extern std::string map_file_; 




const int PARAM_NB_POINTS = 50; // in the trajectory sent to simulator
const double PARAM_DT = 0.02; // 1 point every 0.02 s

const double PARAM_LANE_WIDTH = 4.0; // meters

const double PARAM_MAX_SPEED_MPH = 49;

const double PARAM_MAX_SPEED = 22; // m.s-1
const double PARAM_MAX_ACCEL = 10; // m.s-2
const double PARAM_MAX_JERK  = 10; // m.s-3 average jerk over 1 second

const double PARAM_FOV = 70.0; // Field Of View

const double PARAM_MAX_SPEED_INC = PARAM_MAX_ACCEL * PARAM_DT; // m.s-1 per 0.02 sec
const double PARAM_MAX_SPEED_INC_MPH = ms_to_mph(PARAM_MAX_SPEED_INC);

const double PARAM_DIST_SAFETY = 3.5; // meters

// reduce latency reaction, but account for simulator latency ...
// assume 100 ms max simulator latency
const int PARAM_PREV_PATH_XY_REUSED = 5;

const bool PARAM_TRAJECTORY_JMT = true;

// some extra margin for safety: safety box around the car
const double PARAM_CAR_SAFETY_W = 3; // meters
const double PARAM_CAR_SAFETY_L = 6; // meters
const int PARAM_MAX_COLLISION_STEP = 25;

// cf cost.cpp: weighted cost function from most critical to less critical one
const int PARAM_COST_FEASIBILITY = 10000; // vs collisions, vs vehicle capabilities
const int PARAM_COST_SAFETY      = 1000; // vs buffer distance, vs visibility or curvature
const int PARAM_COST_LEGALITY    = 100; // vs speed limits
const int PARAM_COST_COMFORT     = 10; // vs jerk
const int PARAM_COST_EFFICIENCY  = 1; // vs target lane, target speed and time to goal

const int PARAM_NB_LANES = 3;

// default Safety Distance for Lane Change: will be re-evaluated dynamically
const double PARAM_SD_LC = 10.0;
// default Safety Distance: will be re-evaluated dynamically
const double PARAM_SD = 30.0; */

#endif