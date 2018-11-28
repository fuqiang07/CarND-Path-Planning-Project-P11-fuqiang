#include <math.h>
#include "global.h"

/*
 * **************************************************************** *
 *        Define Helper Functions
 * **************************************************************** *
*/
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
// For converting back and forth between mph and ms.
double mph2ms(double vel_mph) { return vel_mph  / 2.23694; }
double ms2mph(double vel_ms) { return vel_ms * 2.23694; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Get lane ID based on the d value of the car
unsigned int getLaneID(double car_d)
{
    unsigned int uLaneId = 0;

    if ((car_d >= 0.0) and (car_d < GLOBAL_LANE_WIDTH) )
    {
        uLaneId = 1;
    }
    else if ((car_d >= GLOBAL_LANE_WIDTH) and (car_d < GLOBAL_LANE_WIDTH * 2) )
    {
        uLaneId = 2;
    }
    else if ((car_d >= GLOBAL_LANE_WIDTH * 2) and (car_d < GLOBAL_LANE_WIDTH * 3) )
    {
        uLaneId = 3;
    }

    return uLaneId;
}



// d coord for left lane
double get_dleft(int lane) {
  double dleft = lane * PARAM_LANE_WIDTH;
  return dleft;
}

// d coord for right lane
double get_dright(int lane) {
  double dright = (lane + 1) * PARAM_LANE_WIDTH;
  return dright;
}

// d coord for center lane
double get_dcenter(int lane) {
  double dcenter = (lane + 0.5) * PARAM_LANE_WIDTH;
  if (dcenter >= 10) {
    // this a workaround for a simulator issue I think (reported by others as well on udacity forums)
    // with d set to 10 from time to time a lane violation is reported by simulator
    // while everything looks fine
    dcenter = 9.8; // hack !!!
  }
  return dcenter;
}

/* int get_lane(double d) {
  return (int)(d / PARAM_LANE_WIDTH);
} */

