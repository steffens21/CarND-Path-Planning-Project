#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "json.hpp"
#include "tools.h"
#include "spline.h"
#include "veh.h"
#include "path_gen.h"

using namespace std;

// for convenience
using json = nlohmann::json;


bool DEBUG = true;
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



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
      istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
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

  h.onMessage([&map_waypoints_x,
               &map_waypoints_y,
               &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws,
               char *data,
               size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(data);
        if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
                // j[1] is the data JSON object

                // Main car's localization Data
                double car_x = j[1]["x"];
                double car_y = j[1]["y"];
                double car_s = j[1]["s"];
                double car_d = j[1]["d"];
                double car_yaw = j[1]["yaw"];
                double car_speed = j[1]["speed"];

                if (DEBUG) {
                    std::cout << " CAR STATE: (x,y) , (s,d), yaw, speed"
                              << std::endl;
                    std::cout << " " << car_x
                              << " " << car_y
                              << "   " << car_s
                              << " " << car_d
                              << "    " << car_yaw
                              << "   " << car_speed << std::endl;
                }

                // Previous path data given to the Planner
                auto previous_path_x = j[1]["previous_path_x"];
                auto previous_path_y = j[1]["previous_path_y"];
                // Previous path's end s and d values
                double end_path_s = j[1]["end_path_s"];
                double end_path_d = j[1]["end_path_d"];

                // Sensor Fusion Data, a list of all other cars on the same
                // side of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];
                // The data format for each car looks like this,
                // [ id, x, y, vx, vy, s, d].
                // The id is a unique identifier for that car although
                // that same car will always end up in the same position in
                // the vector as well. The x, y values are in global map
                // coordinates, and the vx, vy values are the velocity
                // components also in reference to the global map.
                // Finally s and d are the Frenet coordinates for that car.

                json msgJson;

                // define a path made up of (x,y) points that the car will
                // visit sequentially every .02 seconds


                int path_size = previous_path_x.size();

                vector<double> ptsx;
                vector<double> ptsy;

                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);
                double ref_speed = car_speed;

                // If previous path is small, use car as starting reference
                if (path_size < 2) {
                    double prev_car_x = car_x - cos(car_yaw);
                    double prev_car_y = car_y - sin(car_yaw);

                    // Use 2 points that make the path tangent to the
                    // previous path's end point
                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_x);

                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_y);
                }
                else {
                    ref_x = previous_path_x[path_size - 1];
                    ref_y = previous_path_y[path_size - 1];

                    double ref_x_prev = previous_path_x[path_size - 2];
                    double ref_y_prev = previous_path_y[path_size - 2];
                    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                    // Use 2 points that make the path tangent to the
                    // previous path's end point
                    ptsx.push_back(ref_x_prev);
                    ptsx.push_back(ref_x);
                    
                    ptsy.push_back(ref_y_prev);
                    ptsy.push_back(ref_y);

                    ref_speed = sqrt((ref_x_prev - ref_x)
                                     * (ref_x_prev - ref_x)
                                     + (ref_y_prev - ref_y)
                                     * (ref_y_prev - ref_y)) / 0.02 * 2.237;
                    
                }

                // load vehicle data in more comvenient data container
                vector<Vehicle> other_cars;
                for(int i=0; i<sensor_fusion.size(); i++) {
                    Vehicle veh = Vehicle(sensor_fusion[i][1],
                                          sensor_fusion[i][2],
                                          sensor_fusion[i][3],
                                          sensor_fusion[i][4],
                                          sensor_fusion[i][5],
                                          sensor_fusion[i][6]);
                    other_cars.push_back(veh);
                }

                // determine which lane the car is in
                vector<double> sd = getFrenet(ref_x,
                                              ref_y,
                                              ref_yaw,
                                              map_waypoints_x,
                                              map_waypoints_y);
                double ref_s = sd[0];
                double ref_d = sd[1];
                double ref_sd_yaw = sd[2];

                //std::cout << "ref_sd_yaw " << ref_sd_yaw << std::endl;


                if (DEBUG) {
                    std::cout << "Ref velo: " << ref_speed << std::endl;
                }


                // compare ref_d and car_d to determine if we are changing lanes
                vector<double> result = getTargetSpeedAndLane(ref_s,
                                                              ref_d,
                                                              car_d - ref_d,
                                                              ref_speed,
                                                              other_cars,
                                                              path_size);

                double target_vel = result[0];
                double target_lane = result[1];

                if (DEBUG) {
                    std::cout << "Target velo and lane: " << target_vel << " " << target_lane << std::endl;
                }

                // generate smooth path
                Path p = Path();
                p.generate(target_lane,
                           target_vel,
                           ptsx[0],
                           ptsx[1],
                           ptsy[0],
                           ptsy[1],
                           car_s,
                           ref_yaw,
                           map_waypoints_s,
                           map_waypoints_x,
                           map_waypoints_y);

                // Define the actual (x,y)
                vector<double> next_x_vals;
                vector<double> next_y_vals;

                // re-use previous path
                for(int i = 0; i < path_size; i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }
                
                for(int i = 0; i < 50 - path_size; i++) {
                    next_x_vals.push_back(p.x_vals[i]);
                    next_y_vals.push_back(p.y_vals[i]);
                }
                

                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

                // TODO: try to pass custom info to next iteration
                //msgJson["fooflag"] =

                auto msg = "42[\"control\","+ msgJson.dump()+"]";

                // this_thread::sleep_for(chrono::milliseconds(1000));
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
