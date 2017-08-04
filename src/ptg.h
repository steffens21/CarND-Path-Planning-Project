#ifndef PTG_H_
#define PTG_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "tools.h"
#include "veh.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


class PTG {
    public:
    /**
     * Constructor.
     */
    PTG(bool debug);

    /**
     * Destructor.
     */
    virtual ~PTG();

    bool DEBUG;

    Vehicle vehicle = Vehicle(0, {0,0,0,0,0,0});
    //Vehicle target_vehicle;
    vector<Vehicle> other_cars;
    //vector<double> target_delta;
    //float target_T;

    vector<double> next_s_vals;
    vector<double> next_d_vals;

    // void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    vector<double> JMT(vector<double> start, vector<double> end, double T);

    double poly_eval(vector<double> a, double x);

    void generatePath();
    
    double calculate_cost(vector<double> traj,
                          int target_vehicle,
                          double delta,
                          double goal_T,
                          vector<double> predictions);

    private:

        float target_speed = 15.0; // 25 m/s is about 50 m/h

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


