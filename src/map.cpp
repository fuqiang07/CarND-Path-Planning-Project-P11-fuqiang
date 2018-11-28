#include "map.h"

#include <math.h>
#include <iostream>
#include <sstream>
#include <vector>

#include "global.h"

using namespace std;
using namespace tk;


Map::Map() {}

Map::~Map() {}

/*
* Initialize Map.
*/
void Map::init(const string& map_file, const double& max_s,
 const double& center_x, const double& center_y)
{
	map_file_ = map_file;
	max_s_ = max_s;
	center_x_ = center_x;
	center_y_ = center_y;
}


void Map::read()
{
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
	
	// map with higher precision: 1 point every 1 meter (instead of every 30 meters)
  new_map_waypoints_x;
  new_map_waypoints_y;
  new_map_waypoints_dx;
  new_map_waypoints_dy;
  for (double s = 0; s <= floor(max_s_); s++) {
    double x = spline_x(s);
    double y = spline_y(s);
    double dx = spline_dx(s);
    double dy = spline_dy(s);

    new_map_waypoints_x.push_back(x);
    new_map_waypoints_y.push_back(y);
    new_map_waypoints_dx.push_back(dx);
    new_map_waypoints_dy.push_back(dy);
  }
  
  
  double frenet_s = 0.0;
  map_s.push_back(0.0);
	for (size_t i = 1; i < map_waypoints_x.size(); i++) {
		frenet_s += distance(map_waypoints_x[i], map_waypoints_y[i], map_waypoints_x[i-1], map_waypoints_y[i-1]);
    map_s.push_back(frenet_s);
	}
	
  frenet_s = 0.0;
  new_map_s.push_back(0.0);
  // new map: 1 point every meter
	for (size_t i = 1; i < new_map_waypoints_x.size(); i++) {
		frenet_s += distance(new_map_waypoints_x[i], new_map_waypoints_y[i], new_map_waypoints_x[i-1], new_map_waypoints_y[i-1]);
    //new_map_s.push_back(frenet_s); // TODO test both alternatives
	new_map_s.push_back(i); // better
	}
	
}

int Map::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }
	
    return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta)
{
	vector<double> &maps_s = this->new_map_s; ; 
    vector<double> &maps_x = this->new_map_waypoints_x;
    vector<double> &maps_y = this->new_map_waypoints_y;
  
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = center_x_-maps_x[prev_wp];
    double center_y = center_y_-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
/*     double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    } */
	double frenet_s = maps_s[prev_wp];

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d)
{
	vector<double> &maps_s = map_waypoints_s; ; 
  vector<double> &maps_x = map_waypoints_x;
  vector<double> &maps_y = map_waypoints_y;
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

vector<double> Map::getXYspline(double s, double d) {
  s = fmod(s, max_s_); // bug fix for JMT wraparound
	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);

	return {x,y};
}

double Map::getSpeedToFrenet(double Vxy, double s) {
  s = fmod(s, max_s_);
  double dx_over_ds = spline_x.deriv(1, s);
  double dy_over_ds = spline_y.deriv(1, s);
  double Vs = (Vxy / sqrt(dx_over_ds*dx_over_ds + dy_over_ds*dy_over_ds));
  return Vs;
}