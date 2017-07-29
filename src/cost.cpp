#include <iostream>
#include "cost.h"
#include <vector>

using namespace std;

CostFunctions::CostFunctions() {}

CostFunctions::~CostFunctions() {}


float CostFunctions::time_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::s_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

float CostFunctions::d_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

}

bool CostFunctions::collision_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions) {

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
  
