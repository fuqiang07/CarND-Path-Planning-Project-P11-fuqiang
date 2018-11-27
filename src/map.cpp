#include "map.h"

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
using namespace tk;


Map::Map() {}

Map::~Map() {}

/*
* Initialize Map.
*/
void Map::Init(string map_file) {
	map_file_ = map_file;
	max_s_ = 6945.554;
}


void Map::read() {
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  
  string line;
 
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  while (getline(in_map_, line)) {
    istringstream iss(line);
  	double x, y;
  	float s, d_x, d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
    
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  assert(map_waypoints_x.size() && "map not loaded, probably path include is missing");

  // to get a good spline approximation on last segment wrapping around

    map_waypoints_x.push_back(map_waypoints_x[0]);
	  map_waypoints_y.push_back(map_waypoints_y[0]);
	  map_waypoints_s.push_back(max_s_+map_waypoints_s[0]);
	  map_waypoints_dx.push_back(map_waypoints_dx[0]);
	  map_waypoints_dy.push_back(map_waypoints_dy[0]);

	  map_waypoints_x.push_back(map_waypoints_x[1]);
	  map_waypoints_y.push_back(map_waypoints_y[1]);
	  map_waypoints_s.push_back(max_s_+map_waypoints_s[1]);
	  map_waypoints_dx.push_back(map_waypoints_dx[1]);
	  map_waypoints_dy.push_back(map_waypoints_dy[1]);
  

  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  // remove last point so we do not have duplicates (x,y): it was just for spline continuity at wraparound

    map_waypoints_x.pop_back();
    map_waypoints_y.pop_back();
    map_waypoints_s.pop_back();
    map_waypoints_dx.pop_back();
    map_waypoints_dy.pop_back();

    map_waypoints_x.pop_back();
    map_waypoints_y.pop_back();
    map_waypoints_s.pop_back();
    map_waypoints_dx.pop_back();
    map_waypoints_dy.pop_back();

  double len_ref = 0;
  double prev_x = spline_x(0);
  double prev_y = spline_y(0);
  for (double s = 1; s <= floor(MAX_S); s++) {
    double x = spline_x(s);
    double y = spline_y(s);
    len_ref += distance(x, y, prev_x, prev_y);
    prev_x = x;
    prev_y = y;
  }
  cout << "len_ref=" << len_ref << endl;
}
