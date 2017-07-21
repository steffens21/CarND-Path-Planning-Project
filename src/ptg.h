#ifndef PTG_H_
#define PTG_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


class PTG {
 public:
  /**
   * Constructor.
   */
  PTG();

  /**
   * Destructor.
   */
  virtual ~PTG();

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  vector<double> JMT(vector< double> start, vector <double> end, double T);

  void generatePath(float pos_x,
		    float pos_y,
		    float angle,
		    //TODO: sensor_fusion,
		    vector<double> previous_path_x,
		    vector<double> previous_path_y,
		    double map_waypoints_x,
		    double map_waypoints_y,
		    double map_waypoints_dx,
		    double map_waypoints_dy);
  
 private:

  //bool is_initialized_;
  int N_SAMPLES = 10;
  float SIGMA_S [3] = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
  float SIGMA_D [3] = {1.0, 1.0, 1.0};
  float SIGMA_T = 2.0;

  float MAX_JERK = 10; // m/s/s/s
  float MAX_ACCEL= 10; // m/s/s

  float EXPECTED_JERK_IN_ONE_SEC = 2; // m/s/s
  float EXPECTED_ACC_IN_ONE_SEC = 1; // m/s

  float SPEED_LIMIT = 30;
  float VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection
};

#endif /* PTG_H_ */


