#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "global.h"

#include "map.h"
#include "predictions.h"
#include "behavior.h"
#include "trajectory.h"
#include "cost.h"

// for convenience
using namespace std;
using json = nlohmann::json;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors

    /*
     * **************************************************************** *
     *        My code here: Map
	 * 1. read the map file and load the waypoints
	 * 2. interpolate the waypoints with spline function
	 * 3. provide functions to convert between Frenet and Cartesian coordinates
     * **************************************************************** *
     */
    Map map;
    map.init(GLOBAL_MAP_FILE, GLOBAL_MAX_S, GLOBAL_CENTER_X, GLOBAL_CENTER_Y);
    map.read();

    // Set an index to help record the initialized value
    bool initialized = false;

    // Store car data in a structure
    // x, y, s, d, yaw, speed, speed_target, lane, emergency
    CarData car = CarData(0., 0., 0., 0., 0.,  0., 1.0, 0., false);

    // Store previous s and d paths
    TrajectorySD prev_path_sd;
    /*
     * **************************************************************** *
     */

    h.onMessage([&map, &car, &initialized, &prev_path_sd](uWS::WebSocket<uWS::SERVER> ws,
                                                          char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    car.x = j[1]["x"];
                    car.y = j[1]["y"];
                    car.s = j[1]["s"];
                    car.d = j[1]["d"];
                    car.yaw = j[1]["yaw"];
                    car.speed = j[1]["speed"];

                    TrajectoryXY previous_path_xy;
                    // Previous path data given to the Planner
                    vector<double>  previous_path_x = j[1]["previous_path_x"];
                    vector<double>  previous_path_y = j[1]["previous_path_y"];
                    previous_path_xy.x_vals = previous_path_x;
                    previous_path_xy.y_vals = previous_path_y;

                    // Previous path's end s and d values
                    //double end_path_s = j[1]["end_path_s"];
                    //double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                    /*
                     * **************************************************************** *
                     *        My code here: previous path points
                     * **************************************************************** *
                     */

                    int prev_size = previous_path_xy.x_vals.size();

                    vector<double> frenet_car = map.getFrenet(car.x, car.y, deg2rad(car.yaw));
                    car.s = frenet_car[0];
                    car.d = frenet_car[1];
                    car.lane = get_lane(car.d);

                    Debug("");
                    Debug("Pre-Planning:******************************");
                    Debug("car.x=" << car.x << " car.y=" << car.y << " car.s=" <<
                                   car.s << " car.d=" << car.d);
                    Debug("car.lane=" << car.lane);
                    Debug("car.speed=" << car.speed << " car.speed_target=" << car.speed_target);

                    if (!initialized) {
                        TrajectoryJMT traj_jmt = JMT_init(car.s, car.d);
                        prev_path_sd = traj_jmt.path_sd;
                        initialized = true;
                    }

                    // Keep points of previous generated trajectory before prev_size
                    // Re-generate path points after prev_size
                    PreviousPath previous_path = PreviousPath(previous_path_xy,
                                                              prev_path_sd,
                                                              min(prev_size, GLOBAL_PREV_PATH_XY_REUSED));

                    /*
                     * **************************************************************** *
                     *        My code here: Prediction
                     * **************************************************************** *
                     */
                    // Predict trajectories (over 1 second, i.e, 50 points in total) 
                    // of other cars (cloest front and back cars for each lane, 6 cars in total)
                    Predictions predictions = Predictions(sensor_fusion,
                                                          car,
                                                          GLOBAL_NUM_POINTS);
                    Debug("");
                    Debug("Prediction:******************************");
                    /*
                     * **************************************************************** *
                     *        My code here: Behavior Planning
                     * **************************************************************** *
                     */
                    // Behavior planning for the ego car
                    // Output a target to follow: lane, time duration, velocity, and acceleration
                    Behavior behavior = Behavior(sensor_fusion, car, predictions);
                    vector<Target> targets = behavior.get_targets();
                    Debug("");
                    Debug("Behavior Planning:******************************");
                    Debug("Possible Target Choices: " << targets.size() );
                    /*
                     * **************************************************************** *
                     *        My code here: Trajectory Generation
                     * **************************************************************** *
                     */
                    // Generate optimal trajectory for the ego car to track					 
                    Trajectory trajectory = Trajectory(targets, map, car, previous_path, predictions);
                    Debug("");
                    Debug("Trajectory Generation:******************************");
                    /*
                     * **************************************************************** *
                     *        My code here: Output Optimal Path to the Simulator
                     * **************************************************************** *
                     */
                    // Output the optimal path 					 
                    double min_cost = trajectory.getMinCost();
                    int min_cost_index = trajectory.getMinCostIndex();
                    vector<double> next_x_vals = trajectory.getMinCostTrajectoryXY().x_vals;
                    vector<double> next_y_vals = trajectory.getMinCostTrajectoryXY().y_vals;

                    if (GLOBAL_TRAJECTORY_JMT) {
                        prev_path_sd = trajectory.getMinCostTrajectorySD();
                    }

                    int target_lane = targets[min_cost_index].lane;
                    car.speed_target = targets[min_cost_index].velocity;

                    if (target_lane != car.lane) {
                        Debug( "=====CHANGE LANE=====> to : " << target_lane << " with target speed" << car.speed_target);
                    }
                    else{
                        Debug( "=====KEEP CURRENT LANE=====> as : " << target_lane << " with target speed" << car.speed_target);
                    }

                    /*
                     * **************************************************************** *
                     */

                    json msgJson;

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
