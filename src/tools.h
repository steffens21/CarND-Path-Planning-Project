#ifndef TOOLS_H_
#define TOOLS_H_
#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "spline.h"
#include "veh.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"


double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x,
                    double y,
                    vector<double> maps_x,
                    vector<double> maps_y);

int NextWaypoint(double x,
                 double y,
                 double theta,
                 vector<double> maps_x,
                 vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x,
                         double y,
                         double theta,
                         vector<double> maps_x,
                         vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY_mod(double s,
                         double d,
                         vector<double> maps_s,
                         vector<double> maps_x,
                         vector<double> maps_y);


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s,
                     double d,
                     vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y);

// Transform x,y values to s,d vals and get speed and accell as well
vector<double> getSDpos(double car_x,
                        double car_y,
                        double car_speed,
                        double car_yaw,
                        vector<double> previous_path_x,
                        vector<double> previous_path_y,
                        vector<double> map_waypoints_x,
                        vector<double> map_waypoints_y);

// Transform vehicle state of form [x, y, vx, vy, s, d]
// to state of form [s, s_dot, s_dotdot, d, d_dot, d_dotdot]
vector<double> transVehState(double x,
                             double y,
                             double vx,
                             double vy,
                             double s,
                             double d);

float eval_traj(vector<double> coeffs, double t);

float nearest_approach(Trajectory traj,
                       Vehicle vehicle);

float nearest_approach_to_any_vehicle(Trajectory traj,
                                      vector<Vehicle> vehicles);

void log_vector(vector<double> v);

double logistic(double x);

vector<double> differentiate(vector<double> coeff);

vector<double> JMT(vector< double> start, vector <double> end, double T);

#endif /* TOOLS_H_ */
