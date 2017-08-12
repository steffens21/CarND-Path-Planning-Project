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
vector<double> getXY(double s,
                     double d,
                     vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y);

bool check_collision(double ref_s,
                     double ref_d,
                     vector<Vehicle> other_cars,
                     int steps);

void log_vector(vector<double> v);

double logistic(double x);

vector<double> differentiate(vector<double> coeff);

vector<double> straight_traj(vector< double> start, vector <double> end, double T);

#endif /* TOOLS_H_ */
