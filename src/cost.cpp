#include <iostream>
#include "cost.h"
#include "tools.h"
#include "veh.h"
#include <vector>

using namespace std;

float VEHICLE_RADIUS = 1.5;

CostFunctions::CostFunctions() {}

CostFunctions::~CostFunctions() {}


float CostFunctions::time_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::s_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::d_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

bool CostFunctions::collision_cost(vector<double> traj,
                                   int target_vehicle,
                                   double delta,
                                   double T,
                                   vector<Vehicle> predictions) {
    double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    if(nearest < 2 * VEHICLE_RADIUS) {
      return true;
    }
    return false;
}

float CostFunctions::buffer_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

//float CostFunctions::stays_on_road_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {}

//float CostFunctions::exceeds_speed_limit_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {}

float CostFunctions::efficiency_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::max_accel_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::total_accel_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::max_jerk_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::total_jerk_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}
  
