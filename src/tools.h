#ifndef TOOLS_H_
#define TOOLS_H_
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "veh.h"


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
vector<double> getXY(double s,
                     double d,
                     vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y);

// Check if we get too close to any other vehicle on the raod
bool check_collision(double ref_s,
                     double ref_d,
                     vector<Vehicle> other_cars,
                     int steps);

double collision_dist(double ref_s,
                      double ref_d,
                      vector<Vehicle> other_cars,
                      int steps);

vector<double> getTargetSpeedAndLane(double ref_s,
                                     double ref_d,
                                     double ref_x,
                                     double ref_y,
                                     double d_diff,
                                     double ref_speed,
                                     vector<Vehicle> other_cars,
                                     int path_size,
                                     bool DEBUG);

void log_vector(vector<double> v);

#endif /* TOOLS_H_ */
