#ifndef COST_H_
#define COST_H_
#include <vector>
using namespace std;

class CostFunctions {
 public:
  /**
   * Constructor.
   */
  CostFunctions();

  /**
   * Destructor.
   */
  virtual ~CostFunctions();

  /*
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
   */
  float time_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  /*
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
  */
  float s_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  /*
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
  */
  float d_diff_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  /*
    Binary cost function which penalizes collisions.
  */
  bool collision_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  /*
    Penalizes getting close to other vehicles.
   */
  float buffer_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  //float stays_on_road_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions):

  //float exceeds_speed_limit_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions):

  /*
    Rewards high average speeds.
  */
  float efficiency_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  /*
    
   */
  float max_accel_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  float total_accel_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  float max_jerk_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);

  float total_jerk_cost(vector<double> traj, int target_vehicle, double delta, double T, vector<double> predictions);
  
};

#endif /* COST_H_ */
