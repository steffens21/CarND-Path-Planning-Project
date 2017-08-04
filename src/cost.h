#ifndef COST_H_
#define COST_H_
#include <vector>
#include "veh.h"

using namespace std;


/*
Penalizes trajectories that span a duration which is longer or
shorter than the duration requested.
*/
float time_diff_cost(Trajectory traj,
                     vector<Vehicle> predictions);

/*
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
*/
float s_diff_cost(Trajectory traj,
                  vector<Vehicle> predictions);

/*
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
*/
float d_diff_cost(Trajectory traj,
                  vector<Vehicle> predictions);

/*
    Binary cost function which penalizes collisions.  Will return 1.0 if collision occurs
*/
float collision_cost(Trajectory traj,
                     vector<Vehicle> predictions);

/*
    Penalizes getting close to other vehicles.
*/
float buffer_cost(Trajectory traj,
                  vector<Vehicle> predictions);

float stays_on_road_cost(Trajectory traj,
                         vector<Vehicle> predictions);

float exceeds_speed_limit_cost(Trajectory traj,
                               vector<Vehicle> predictions);

/*
    Rewards high average speeds.
*/
float efficiency_cost(Trajectory traj,
                      vector<Vehicle> predictions);

  /*
    
   */
float max_accel_cost(Trajectory traj,
                     vector<Vehicle> predictions);

float total_accel_cost(Trajectory traj,
                       vector<Vehicle> predictions);

float max_jerk_cost(Trajectory traj,
                    vector<Vehicle> predictions);

float total_jerk_cost(Trajectory traj,
                      vector<Vehicle> predictions);
  

#endif /* COST_H_ */
