#include "global.h"

#include <limits>       // std::numeric_limits

using namespace std;

/*
 * **************************************************************** *
 *        Define Global Constants and Variables
 * **************************************************************** *
*/
//maximum double value
const double GLOBAL_MAX_DOUBLE = numeric_limits<double>::max();

//the number of points in the trajectory sent to simulator
const int GLOBAL_NUM_POINTS = 50;
//sampling time in this simulation
const double GLOBAL_TS = 0.02;

// define lane width as 4 m 
const double GLOBAL_LANE_WIDTH = 4.0; // meters
// Waypoint map to read from
const string GLOBAL_MAP_FILE = "../data/highway_map.csv";
// The max s value before wrapping around the track back to 0
const double GLOBAL_MAX_S = 6945.554;

// center point of the track, will be used in Map::getFrenet()
const double GLOBAL_CENTER_X = 1000;
const double GLOBAL_CENTER_Y = 2000;
//the number of lane in the simulation
//const int GLOBAL_NUM_LANES = 3; 

//maximum speed in mph
const double GLOBAL_MAX_SPEED_MPH = 49;
//maximum speed in ms
const double GLOBAL_MAX_SPEED = 22;
//maximum acc in ms2	
const double GLOBAL_MAX_ACCEL = 10;
//maximum jerk in ms3		
const double GLOBAL_MAX_JERK  = 10;
//speed increment in ms2
const double GLOBAL_MAX_SPEED_INC = GLOBAL_MAX_ACCEL * GLOBAL_TS;
//speed increment in mph
const double GLOBAL_MAX_SPEED_INC_MPH = ms2mph(GLOBAL_MAX_SPEED_INC);

// default Safety Distance for Lane Change: will be re-evaluated dynamically
const double GLOBAL_SD_LC = 10.0;
// default Safety Distance for current lane: will be re-evaluated dynamically
const double GLOBAL_SD = 30.0;

// Field Of View
const double GLOBAL_FIELD_OF_VIEW = 70.0;

// reduce latency reaction, but account for simulator latency ...
// assume 100 ms max simulator latency
const int GLOBAL_PREV_PATH_XY_REUSED = 5;

const bool GLOBAL_TRAJECTORY_JMT = true;

// some extra margin for safety: safety box around the car
const double GLOBAL_CAR_SAFETY_W = 3; // meters
const double GLOBAL_CAR_SAFETY_L = 6; // meters
const int GLOBAL_MAX_COLLISION_STEP = 25;

// cf cost.cpp: weighted cost function from most critical to less critical one
const int GLOBAL_COST_FEASIBILITY = 10000; // vs collisions, vs vehicle capabilities
const int GLOBAL_COST_SAFETY      = 1000; // vs buffer distance, vs visibility or curvature
const int GLOBAL_COST_LEGALITY    = 100; // vs speed limits
const int GLOBAL_COST_COMFORT     = 10; // vs jerk
const int GLOBAL_COST_EFFICIENCY  = 1; // vs target lane, target speed and time to goal

/*
 * **************************************************************** *
 *        Define Helper Functions
 * **************************************************************** *
*/
// For converting back and forth between radians and degrees.
double deg2rad(double x){
    return x * pi() / 180;
}
double rad2deg(double x){
    return x * 180 / pi();
}

// For converting back and forth between mph and ms.
double mph2ms(double vel_mph){
    return vel_mph  / 2.23694;
}
double ms2mph(double vel_ms){
    return vel_ms * 2.23694;
}

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2){
    double dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    return dist;
}

// Get lane ID based on the d value of the car
// 0 -- left lane; 1 -- middle lane; 2 -- right lane
unsigned int get_lane(double d){
    unsigned int lane_id = (unsigned int)(d / GLOBAL_LANE_WIDTH);
    return lane_id;
}

// Get the d value for the left edge of the current lane
double get_dleft(int lane_id) {
    double dleft = lane_id * GLOBAL_LANE_WIDTH;
    return dleft;
}

// Get the d value for the right edge of the current lane
double get_dright(int lane_id) {
    double dright = (lane_id + 1) * GLOBAL_LANE_WIDTH;
    return dright;
}

// Get the d value for the center of the current lane
double get_dcenter(int lane_id) {
    double dcenter = (lane_id + 0.5) * GLOBAL_LANE_WIDTH;
    if (dcenter >= 10) {
        // this a workaround for a simulator issue I think (reported by others as well on udacity forums)
        // with d set to 10 from time to time a lane violation is reported by simulator
        // while everything looks fine
        dcenter = 9.8; // hack !!!
    }
    return dcenter;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}