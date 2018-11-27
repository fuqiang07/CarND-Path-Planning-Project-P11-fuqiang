#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "spline.h"

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

  /*
  * Initialize MAP.
  */
  void Init(string map_file);
  
  void read();
  
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
};

#endif /* MAP_H */