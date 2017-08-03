#include <iostream>
#include <vector>
#include "tools.h"
#include "veh.h"
#include "cost.h"

using namespace std;

float VEHICLE_RADIUS = 1.5;

CostFunctions::CostFunctions() {}

CostFunctions::~CostFunctions() {}


float CostFunctions::time_diff_cost(Trajectory traj,
                                    int target_vehicle,
                                    double delta,
                                    double T,
                                    vector<double> predictions) {
    return 0.0;
}

float CostFunctions::s_diff_cost(Trajectory traj,
                                 int target_vehicle,
                                 double delta,
                                 double T,
                                 vector<double> predictions) {
    return 0.0;
}

float CostFunctions::d_diff_cost(Trajectory traj,
                                 int target_vehicle,
                                 double delta,
                                 double T,
                                 vector<double> predictions) {
    return 0.0;
}

float CostFunctions::collision_cost(Trajectory traj,
                                   int target_vehicle,
                                   double delta,
                                   double T,
                                   vector<Vehicle> predictions) {
    double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    if(nearest < 2 * VEHICLE_RADIUS) {
        return 1.0;
    }
    return 0.0;
}

float CostFunctions::buffer_cost(Trajectory traj,
                                 int target_vehicle,
                                 double delta,
                                 double T,
                                 vector<double> predictions) {
    return 0.0;
}

//float CostFunctions::stays_on_road_cost(Trajectory traj, int target_vehicle, double delta, double T, vector<double> predictions) {}

//float CostFunctions::exceeds_speed_limit_cost(Trajectory traj, int target_vehicle, double delta, double T, vector<double> predictions) {}

float CostFunctions::efficiency_cost(Trajectory traj,
                                     int target_vehicle,
                                     double delta,
                                     double T,
                                     vector<double> predictions) {
    return 0.0;
}

float CostFunctions::max_accel_cost(Trajectory traj,
                                    int target_vehicle,
                                    double delta,
                                    double T,
                                    vector<double> predictions) {
    return 0.0;
}

float CostFunctions::total_accel_cost(Trajectory traj,
                                      int target_vehicle,
                                      double delta,
                                      double T,
                                      vector<double> predictions) {
    return 0.0;
}

float CostFunctions::max_jerk_cost(Trajectory traj,
                                   int target_vehicle,
                                   double delta,
                                   double T,
                                   vector<double> predictions) {
    return 0.0;
}

float CostFunctions::total_jerk_cost(Trajectory traj,
                                     int target_vehicle,
                                     double delta,
                                     double T,
                                     vector<double> predictions) {
    return 0.0;
}
  
