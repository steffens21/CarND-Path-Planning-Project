#ifndef PTG_H_
#define PTG_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "tools.h"
#include "cost.h"
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
    vector<Vehicle> other_cars;

    vector<double> next_s_vals;
    vector<double> next_d_vals;

    double poly_eval(vector<double> a, double x);

    void generatePath();
    
    double calculate_cost(Trajectory traj);

    private:

        float target_speed = 15.0; // 25 m/s is about 50 m/h

        int N_SAMPLES = 10;
        float SIGMA_S [3] = {10.0, 4.0, 2.0}; // s, s_dot, s_double_dot
        float SIGMA_D [3] = {1.0, 1.0, 1.0};
        float SIGMA_T = 2.0;

        float EXPECTED_JERK_IN_ONE_SEC = 2; // m/s/s

        float SPEED_LIMIT = 30;
        float VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection
};

#endif /* PTG_H_ */


