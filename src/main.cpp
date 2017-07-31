#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "ptg.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x,
                    double y,
                    vector<double> maps_x,
                    vector<double> maps_y) {

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

int NextWaypoint(double x,
                 double y,
                 double theta,
                 vector<double> maps_x,
                 vector<double> maps_y) {

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x,
                         double y,
                         double theta,
                         vector<double> maps_x,
                         vector<double> maps_y) {
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

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY_mod(double s,
                         double d,
                         vector<double> maps_s,
                         vector<double> maps_x,
                         vector<double> maps_y) {

    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    if (prev_wp == -1) {
        prev_wp = 0;
    }

    // interpolate between wp3 and prev_wp
    int wp2 = (prev_wp + 1) % maps_x.size();
    int wp3 = (wp2 + 1) % maps_x.size();

    tk::spline spline_s_x;
    tk::spline spline_s_y;
    vector<double> maps_x_sel = {maps_x[prev_wp], maps_x[wp2], maps_x[wp3]};
    vector<double> maps_y_sel = {maps_y[prev_wp], maps_y[wp2], maps_y[wp3]};
    vector<double> maps_s_sel = {maps_s[prev_wp], maps_s[wp2], maps_s[wp3]};
    
    vector<double> maps_x_mod;
    vector<double> maps_y_mod;
    vector<double> maps_s_mod;

    
    spline_s_x.set_points(maps_s_sel,
                          maps_x_sel);
    spline_s_y.set_points(maps_s_sel,
                          maps_y_sel);
    
    int samples = 40;
    float step_s = abs(maps_s_sel[-1] - maps_s_sel[0]) / float(samples);
    for (int i = 0; i < samples; i++) {
        float s_val = maps_s_sel[0] + i * step_s;
        maps_x_mod.push_back(spline_s_x(s_val));
        maps_y_mod.push_back(spline_s_y(s_val));
        maps_s_mod.push_back(s_val);
    }

    // now perform original getXY calculation with refined waypoints
    int prev_wp_mod = -1;
    while(s > maps_s_mod[prev_wp_mod+1]
          && (prev_wp_mod < (int)(maps_s_mod.size()-1))) {
        prev_wp_mod++;
    }
    // the modulus should never be applied
    int wp2_mod = (prev_wp_mod + 1) % maps_x_mod.size();

    double heading = atan2((maps_y_mod[wp2_mod]-maps_y_mod[prev_wp_mod]),
                           (maps_x_mod[wp2_mod]-maps_x_mod[prev_wp_mod]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s_mod[prev_wp_mod]);

    double seg_x = maps_x_mod[prev_wp_mod] + seg_s * cos(heading);
    double seg_y = maps_y_mod[prev_wp_mod] + seg_s * sin(heading);

    double perp_heading = heading - pi()/2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x,y};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s,
                     double d,
                     vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y) {
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                           (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading-pi() / 2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
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
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

                json msgJson;

                // define a path made up of (x,y) points that the car will
                // visit sequentially every .02 seconds

                double pos_x;
                double pos_y;
                double angle;
                int path_size = previous_path_x.size();

                int NBR_PRED_POINTS = 50;

                double pos_speed;
                double pos_accell;
                double pos_x2;
                double pos_y2;

                if(path_size == 0) {
                    pos_x = car_x;
                    pos_y = car_y;
                    pos_x2 = pos_x;
                    pos_y2 = pos_y;
                    angle = deg2rad(car_yaw);
                    vector<double> result = getFrenet(pos_x,
                                                      pos_y,
                                                      angle,
                                                      map_waypoints_x,
                                                      map_waypoints_y);
                    end_path_s = result[0];
                    end_path_d = result[1];
                    //double car_accel = 0.0;
                    pos_speed = car_speed * 0.44704;
                    pos_accell = 0;
                }
                else {
                    pos_x = previous_path_x[path_size-1];
                    pos_y = previous_path_y[path_size-1];

                    if(path_size > 1) {
                        pos_x2 = previous_path_x[path_size-2];
                        pos_y2 = previous_path_y[path_size-2];
                        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
                        pos_speed = sqrt(pow(pos_x-pos_x2, 2)
                                         + pow(pos_y-pos_y2,2)) / 0.02;
                        pos_speed = min(25.0, pos_speed);
                    }
                    else {
                        angle = deg2rad(car_yaw);
                        pos_speed = car_speed * 0.44704;
                    }

                    if(path_size > 2) {
                        double pos_x3 = previous_path_x[path_size - 3];
                        double pos_y3 = previous_path_y[path_size - 3];
                        double pos_speed2 = sqrt((pos_x2 - pos_x3)
                                                      * (pos_x2 - pos_x3)
                                                 + (pos_y2 - pos_y3)
                                                      * (pos_y2 - pos_y3)
                                                 ) / 0.02;
                        pos_accell = pos_speed2 - pos_speed;
                    }
                    else {
                        pos_accell = 0;
                    }
                }

                // Create a path trajectory generator (PTG) instance
                PTG ptg(DEBUG);

                // Use helper function to get next waypoint
                int next_wayp = NextWaypoint(car_x,
                                             car_y,
                                             deg2rad(car_yaw),
                                             map_waypoints_x,
                                             map_waypoints_y);
                int pos_next_wayp = NextWaypoint(pos_x,
                                                 pos_y,
                                                 angle,
                                                 map_waypoints_x,
                                                 map_waypoints_y);
	
                ptg.generatePath(pos_x,
                                 pos_y,
                                 //car_s,
                                 //car_d,
                                 //car_yaw,
                                 pos_speed,
                                 pos_accell,
                                 angle,
                                 end_path_s,
                                 end_path_d,
                                 // TODO: sensor_fusion,
                                 map_waypoints_x[pos_next_wayp],
                                 map_waypoints_y[pos_next_wayp],
                                 map_waypoints_s[pos_next_wayp],
                                 map_waypoints_dx[pos_next_wayp],
                                 map_waypoints_dy[pos_next_wayp],
                                 NBR_PRED_POINTS - path_size);

                vector<double> next_x_vals;
                vector<double> next_y_vals;

                for(int i = 0; i < path_size; i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }

                for (int i=0; i < ptg.next_s_vals.size(); i++) {
                    // TODO: use ptg.next_d_vals[i]
                    vector<double> result = getXY(ptg.next_s_vals[i],
                                                  end_path_d,
                                                  map_waypoints_s,
                                                  map_waypoints_x,
                                                  map_waypoints_y);
                    if (next_x_vals.size() > 0
                        && result[0] == next_x_vals[path_size-1]) {
                        continue;
                    }
                    next_x_vals.push_back(result[0]);
                    next_y_vals.push_back(result[1]);
                    if (DEBUG) {
                        std::cout << result[0] << " " << result[1] << std::endl;
                    }
                }

                if (DEBUG) {
                    for(int i=0; i<next_x_vals.size();i++) {
                        std::cout << "     *** " << next_x_vals[i]
                        << " " << next_y_vals[i]
                        << " " << std::endl;
                        if (i>0) {
                            double speed = sqrt(
                                                pow(next_x_vals[i] - next_x_vals[i-1], 2)
                                                + pow(next_y_vals[i] - next_y_vals[i-1], 2)
                                                ) / 0.02;
                            if (speed > 25) {
                                std::cout << "BAD!!! " << i
                                << " " << ptg.next_s_vals[i]
                                << " " << ptg.next_s_vals[i-1]
                                << " " << end_path_d << std::endl;
                            }
                            if (speed < 5) {
                                std::cout << "BAD!!! " << i << std::endl;
                            }
                            std::cout << "                                  "
                            << speed  << std::endl;
                        }
                    }
                }

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
















































































