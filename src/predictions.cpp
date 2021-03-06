#include "predictions.h"


using namespace std;


//Step 1: Find the cloest vehicles (ahead and back) around the ego one for each lane, 6 cars in total
//Step 2: Get the data of those cloest vehicles
Predictions::Predictions(vector<vector<double>> const &sensor_fusion, CarData const &car, int horizon)
{
    // Get the index of the cloest vehicles
    vector<int> closest_objects = find_closest_vehicles_ID(sensor_fusion, car);

    // Predict the future trajectories based on car Model
    for (int i = 0; i < closest_objects.size(); i++) {
        int fusion_index = closest_objects[i];
        if (fusion_index >= 0) {
            double x = sensor_fusion[fusion_index][1];
            double y = sensor_fusion[fusion_index][2];
            double vx = sensor_fusion[fusion_index][3];
            double vy = sensor_fusion[fusion_index][4];
            vector<Coord> prediction; // vector of at most 6 predicitons of "n_horizon" (x,y) coordinates
            for (int j = 0; j < horizon; j++) {
                Coord coord;
                coord.x = x + vx * j*GLOBAL_TS;
                coord.y = y + vy * j*GLOBAL_TS;
                prediction.push_back(coord);
            }
            predictions_[fusion_index] = prediction;
        }
    }

    set_safety_distances(sensor_fusion, car);
    set_lane_info(sensor_fusion, car);
}

Predictions::~Predictions() {}


double get_sensor_fusion_vel(vector<vector<double>> const &sensor_fusion, int idx, double default_vel) {
	
	double vx, vy, vel;
	
	if (idx >= 0 && idx < sensor_fusion.size()) {
		vx = sensor_fusion[idx][3];
		vy = sensor_fusion[idx][4];
		vel = sqrt(vx*vx+vy*vy);
	} 
	else {
		vel = default_vel;
	}
	
	return vel;
}

double Predictions::get_safety_distance(double vel_back, double vel_front, double time_latency)
{
    double safety_distance = GLOBAL_SD_LC;
    if (vel_back > vel_front) {
        double time_to_decelerate = (vel_back - vel_front) / decel_ + time_latency;
        safety_distance = vel_back * time_to_decelerate + 1.5 * GLOBAL_CAR_SAFETY_L;
    }
    safety_distance = max(safety_distance, GLOBAL_SD_LC);
	
    return safety_distance;
}

void Predictions::set_safety_distances(vector<vector<double>> const &sensor_fusion, CarData const &car)
{
    vel_ego_ = mph2ms(car.speed);  // velocity of ego vehicle
    // slightly conservative as it will relate to safety distance
    decel_ = 0.8 * GLOBAL_MAX_ACCEL;
    time_to_stop_ = vel_ego_ / decel_;

    vel_front_ = get_sensor_fusion_vel(sensor_fusion, front_id_[car.lane], GLOBAL_MAX_SPEED);
    dist_front_ = front_dmin_[car.lane];

    if (vel_ego_ > vel_front_) {
        time_to_collision_ = dist_front_ / (vel_ego_ - vel_front_);
        time_to_decelerate_ = (vel_ego_ - vel_front_) / decel_;
        safety_distance_ = vel_ego_ * time_to_decelerate_ + 1.75 * GLOBAL_CAR_SAFETY_L;
    } else {
        time_to_collision_ = GLOBAL_MAX_DOUBLE;
        time_to_decelerate_ = 0;
        safety_distance_ = 1.75 * GLOBAL_CAR_SAFETY_L;
    }

    paranoid_safety_distance_ = vel_ego_ * time_to_stop_ + 2 * GLOBAL_CAR_SAFETY_L;

    for (int i = 0; i < GLOBAL_NUM_LANES; i++) {
        front_velocity_[i] = get_sensor_fusion_vel(sensor_fusion, front_id_[i], GLOBAL_MAX_SPEED);
        front_safety_distance_[i] = get_safety_distance(vel_ego_, front_velocity_[i], 0.0);

        back_velocity_[i] = get_sensor_fusion_vel(sensor_fusion, back_id_[i], 0);
        back_safety_distance_[i] = get_safety_distance(back_velocity_[i], vel_ego_, 2.0);
    }

}

void Predictions::set_lane_info(vector<vector<double>> const &sensor_fusion, CarData const &car)
{
    int car_lane = get_lane(car.d);
    for (size_t i = 0; i < front_id_.size(); i++) {
        //cout << "lane " << i << ": ";
        //cout << "front " << front_id_[i] << " at " << front_dmin_[i] << " s_meters ; ";
        //cout << "back " << back_id_[i] << " at " << back_dmin_[i] << " s_meters" << endl;

        int lane = i;
        // !!! This should be part of the behavior planner behavior.cpp
        if (front_id_[i] >= 0) { // a car in front of us
            //if (lane != car_lane && (back_dmin_[i] <= 10 || front_dmin_[i] <= 10)) {
            if (lane != car_lane && (back_dmin_[i] <= back_safety_distance_[i] || front_dmin_[i] <= front_safety_distance_[i])) {
                lane_speed_[i] = 0;
                lane_free_space_[i] = 0; // too dangerous
            } else {
                double vx = sensor_fusion[front_id_[i]][3];
                double vy = sensor_fusion[front_id_[i]][4];
                lane_speed_[i] = sqrt(vx*vx+vy*vy);
                lane_free_space_[i] = front_dmin_[i];
            }
        } else {  // if nobody in front of us
            //if (lane != car_lane && back_dmin_[i] <= 10) {
            if (lane != car_lane && back_dmin_[i] <= back_safety_distance_[i]) {
                lane_speed_[i] = 0;
                lane_free_space_[i] = 0; // too dangerous
            } else {
                lane_speed_[i] = GLOBAL_MAX_SPEED_MPH;
                lane_free_space_[i] = GLOBAL_FIELD_OF_VIEW;
            }
        }
        //cout << "Predictions::lane_speed_[" << i << "]=" << lane_speed_[i] << endl;
    }
}

// Find the cloest vehicles based on sensor function and localization data
// we only care about the cloest vehicles front and back for each lane
// that is to say, front 0 and back 0, front 1 and back 1, front 2 and back 2 for lane 0, 1, 2, respectively
vector<int> Predictions::find_closest_vehicles_ID(vector<vector<double>> const &sensor_fusion, CarData const &car) {

    // we only consider the vehicles within the field of view: 70 meters
    double sfov_min = car.s - GLOBAL_FIELD_OF_VIEW;
    double sfov_max = car.s + GLOBAL_FIELD_OF_VIEW;

    // handle the cases at the very begining / end of the waypoints
    double sfov_shit = 0;
    if (sfov_min < 0) { // Handle s wrapping
        sfov_shit = -sfov_min;
    }
    else if (sfov_max > GLOBAL_MAX_S) {
        sfov_shit = GLOBAL_MAX_S - sfov_max;
    }

    sfov_min += sfov_shit;
    assert(sfov_min >= 0 && sfov_min <= GLOBAL_MAX_S);

    sfov_max += sfov_shit;
    assert(sfov_max >= 0 && sfov_max <= GLOBAL_MAX_S);

    double car_s = car.s + sfov_shit;

    for (size_t i = 0; i < sensor_fusion.size(); i++) {

        double s = sensor_fusion[i][5] + sfov_shit;

        // vehicle in FOV
        if (s >= sfov_min && s <= sfov_max) {

            double d = sensor_fusion[i][6];
            int lane = get_lane(d);

            if (lane < 0 || lane > 2)
                continue; // drop some exceptional data

            double dist = fabs(s - car_s);

            if (s >= car_s) {  // front
                if (dist < front_dmin_[lane]) {
                    front_id_[lane] = i;
                    front_dmin_[lane] = dist;
                }
            }
            else {  // back
                if (dist < back_dmin_[lane]) {
                    back_id_[lane] = i;
                    back_dmin_[lane] = dist;
                }
            }
        }
    }


    return { front_id_[0], back_id_[0], front_id_[1], back_id_[1], front_id_[2], back_id_[2] };
}

double Predictions::get_lane_speed(int lane) const {
    if (lane >= 0 && lane <= 3) {
        return lane_speed_[lane];
    } else {
        return 0;
    }
}

double Predictions::get_lane_free_space(int lane) const {
    if (lane >= 0 && lane <= 3) {
        return lane_free_space_[lane];
    } else {
        return 0;
    }
}
