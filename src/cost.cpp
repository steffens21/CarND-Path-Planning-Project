#include <iostream>
#include <vector>
#include "tools.h"
#include "veh.h"
#include "cost.h"

using namespace std;

float VEHICLE_RADIUS = 1.5;



float time_diff_cost(Trajectory traj,
                                    int target_vehicle,
                                    double delta,
                                    double T,
                                    vector<double> predictions) {
    return 0.0;
}

float s_diff_cost(Trajectory traj,
                                 int target_vehicle,
                                 double delta,
                                 double T,
                                 vector<double> predictions) {
    return 0.0;
}

float d_diff_cost(Trajectory traj,
                                 int target_vehicle,
                                 double delta,
                                 double T,
                                 vector<double> predictions) {
    return 0.0;
}

float collision_cost(Trajectory traj,
                                    //int target_vehicle,
                                    //double delta,
                                    //double T,
                                   vector<Vehicle> predictions) {
    double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    if(nearest < 2 * VEHICLE_RADIUS) {
        return 1.0;
    }
    return 0.0;
}

float buffer_cost(Trajectory traj,
                                 int target_vehicle,
                                 double delta,
                                 double T,
                                 vector<double> predictions) {
    return 0.0;
}

//float stays_on_road_cost(Trajectory traj, int target_vehicle, double delta, double T, vector<double> predictions) {}

//float exceeds_speed_limit_cost(Trajectory traj, int target_vehicle, double delta, double T, vector<double> predictions) {}

float efficiency_cost(Trajectory traj,
                                     int target_vehicle,
                                     double delta,
                                     double T,
                                     vector<double> predictions) {
    return 0.0;
}

float max_accel_cost(Trajectory traj,
                                    int target_vehicle,
                                    double delta,
                                    double T,
                                    vector<double> predictions) {
    return 0.0;
}

float total_accel_cost(Trajectory traj,
                                      int target_vehicle,
                                      double delta,
                                      double T,
                                      vector<double> predictions) {
    return 0.0;
}

float max_jerk_cost(Trajectory traj,
                                   int target_vehicle,
                                   double delta,
                                   double T,
                                   vector<double> predictions) {
    return 0.0;
}

float total_jerk_cost(Trajectory traj,
                                     int target_vehicle,
                                     double delta,
                                     double T,
                                     vector<double> predictions) {
    return 0.0;
}
  
