#include <iostream>
#include <vector>
#include "tools.h"
#include "veh.h"
#include "cost.h"

using namespace std;

float VEHICLE_RADIUS = 1.5;

float time_diff_cost(Trajectory traj,
                     vector<Vehicle> predictions) {
    return 0.0;
}

float s_diff_cost(Trajectory traj,
                  vector<Vehicle> predictions) {
    return 0.0;
}

float d_diff_cost(Trajectory traj,
                  vector<Vehicle> predictions) {
    return 0.0;
}

float collision_cost(Trajectory traj,
                     vector<Vehicle> predictions) {
    double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    if(nearest < 2 * VEHICLE_RADIUS) {
        return 1.0;
    }
    return 0.0;
}

float buffer_cost(Trajectory traj,
                  vector<Vehicle> predictions) {
    return 0.0;
}

float stays_on_road_cost(Trajectory traj,
                         vector<Vehicle> predictions) {
    return 0.0;
}

float exceeds_speed_limit_cost(Trajectory traj,
                               vector<Vehicle> predictions) {
    return 0.0;
}

float efficiency_cost(Trajectory traj,
                      vector<Vehicle> predictions) {
    return 0.0;
}

float total_accel_cost(Trajectory traj,
                     vector<Vehicle> predictions) {
    vector<double> s_dot_coeff = differentiate(traj.s_coeff);
    vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);
    double total_acc = 0.0;
    for (int i=0; i<100; i++){
        float acc = eval_traj(s_dot_dot_coeff, traj.t / 100.0 * i);
        total_acc += abs(acc);
    }
    double acc_per_sec = total_acc / traj.t;
    float EXPECTED_ACC_IN_ONE_SEC = 1;
    return logistic(acc_per_sec / EXPECTED_ACC_IN_ONE_SEC);
}

float max_accel_cost(Trajectory traj,
                       vector<Vehicle> predictions) {
    vector<double> s_dot_coeff = differentiate(traj.s_coeff);
    vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);
    float max_acc = 0.0;
    for (int i=0; i<100; i++){
        float acc = eval_traj(s_dot_dot_coeff, traj.t / 100.0 * i);
        if (abs(acc) > max_acc){
            max_acc = acc;
        }
    }
    float MAX_ACCEL= 10; // m/s/s
    if (max_acc > MAX_ACCEL) {
        return 1.0;
    }
    return 0.0;
}

float max_jerk_cost(Trajectory traj,
                    vector<Vehicle> predictions) {
    vector<double> s_dot_coeff = differentiate(traj.s_coeff);
    vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);
    vector<double> jerk_coeff = differentiate(s_dot_dot_coeff);
    float max_jerk = 0.0;
    for (int i=0; i<100; i++){
        float jerk = eval_traj(jerk_coeff, traj.t / 100.0 * i);
        if (abs(jerk) > max_jerk){
            max_jerk = jerk;
        }
    }
    float MAX_JERK = 10; // m/s/s/s
    if (max_jerk > MAX_JERK) {
        return 1.0;
    }
    return 0.0;
}

float total_jerk_cost(Trajectory traj,
                      vector<Vehicle> predictions) {
    vector<double> s_dot_coeff = differentiate(traj.s_coeff);
    vector<double> s_dot_dot_coeff = differentiate(s_dot_coeff);
    vector<double> jerk_coeff = differentiate(s_dot_dot_coeff);
    double total_jerk = 0.0;
    log_vector(jerk_coeff);
    for (int i=0; i<100; i++){
        float jerk = eval_traj(jerk_coeff, traj.t / 100.0 * i);
        std::cout << jerk << std::endl;
        total_jerk += abs(jerk);
    }
    double jerk_per_sec = total_jerk / traj.t;
    float EXPECTED_JERK_IN_ONE_SEC = 1;
    return logistic(jerk_per_sec / EXPECTED_JERK_IN_ONE_SEC);
}
  
