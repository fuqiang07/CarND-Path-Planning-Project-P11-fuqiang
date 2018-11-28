#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>

#include "global.h"

// TODO remove hardcoded values

class Predictions {
public:
    Predictions(std::vector<std::vector<double> > const &sensor_fusion, CarData const &car, int horizon);
    virtual ~Predictions();

    std::map<int, std::vector<Coord> > get_predictions() const { return predictions_; };
    double get_safety_distance() const { return safety_distance_; };
    double get_paranoid_safety_distance() const { return paranoid_safety_distance_; };
    double get_lane_speed(int lane) const;
    double get_lane_free_space(int lane) const;


private:
    void set_safety_distances(std::vector<std::vector<double>> const &sensor_fusion, CarData const &car);
    void set_lane_info(std::vector<std::vector<double>> const &sensor_fusion, CarData const &car);
    std::vector<int> find_closest_objects(std::vector<std::vector<double>> const &sensor_fusion, CarData const &car);
    double get_safety_distance(double vel_back, double vel_front, double time_latency);

    // TODO use vector init depending on GLOBAL_NUM_LANES
    std::vector<int> front_= {-1, -1, -1};  // idx of closest object per lane
    std::vector<int> back_ = {-1, -1, -1};   // idx of closest object per lane

    // TODO use FOV instead of GLOBAL_MAX_DOUBLE
    std::vector<double> front_dmin_ = {GLOBAL_MAX_DOUBLE, GLOBAL_MAX_DOUBLE, GLOBAL_MAX_DOUBLE};  // dist min per lane
    std::vector<double> back_dmin_ = {GLOBAL_MAX_DOUBLE, GLOBAL_MAX_DOUBLE, GLOBAL_MAX_DOUBLE};   // dist min per lane

    std::vector<double> front_velocity_ = {GLOBAL_MAX_SPEED, GLOBAL_MAX_SPEED, GLOBAL_MAX_SPEED};
    std::vector<double> front_safety_distance_ = {GLOBAL_SD_LC, GLOBAL_SD_LC, GLOBAL_SD_LC};

    std::vector<double> back_velocity_ = {GLOBAL_MAX_SPEED, GLOBAL_MAX_SPEED, GLOBAL_MAX_SPEED};
    std::vector<double> back_safety_distance_ = {GLOBAL_SD_LC, GLOBAL_SD_LC, GLOBAL_SD_LC};

    // map of at most 6 predicitons of "n_horizon" (x,y) coordinates
    std::map< int, std::vector<Coord> > predictions_;
    double lane_speed_[GLOBAL_NUM_LANES];
    double lane_free_space_[GLOBAL_NUM_LANES];

    // safety distance computation related
    double vel_ego_;
    double decel_;

    double dist_front_;
    double vel_front_;
    double time_to_collision_;  // vs front vehicle
    double time_to_stop_;  // time from vel_ego_ to 0
    double time_to_decelerate_;  // time from vel_ego_ to vel_front_

    // will be used by behavior planner
    double safety_distance_ = GLOBAL_SD;
    double paranoid_safety_distance_ = GLOBAL_SD;

};

#endif // PREDICTIONS_H
