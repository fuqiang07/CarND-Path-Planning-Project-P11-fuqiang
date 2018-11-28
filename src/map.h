#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "spline.h"
#include "global.h"

using namespace std;
using namespace tk;

class Map {
 public:
 /*
  * Constructor
  */
  Map();

  /*
  * Destructor.
  */
  virtual ~Map();

  string map_file_;
  double max_s_;
  
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  spline spline_x;
  spline spline_y;
  spline spline_dx;
  spline spline_dy;
  
  vector<double> map_s; // pre-computed for faster access

  // better granularity: 1 point per meter
  vector<double> new_map_waypoints_x;
  vector<double> new_map_waypoints_y;
  vector<double> new_map_waypoints_dx;
  vector<double> new_map_waypoints_dy;

  vector<double> new_map_s; // pre-computed for faster access

  
  /*
  * Initialize MAP.
  */
  void init(const string& map_file, const double& max_s);
   /*
  * Read MAP from file.
  */ 
  void read();
   /*
  * Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  */ 
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  vector<double> getFrenet(double x, double y, double theta);
  vector<double> getXY(double s, double d);
  vector<double> getXYspline(double s, double d); // with splines
  double getSpeedToFrenet(double Vxy, double s);

};

#endif /* MAP_H */