#include <iostream>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "veh.h"
#include <vector>

using namespace std;

Vehicle::Vehicle(vector<double> start_state) {
    id = start_state[0];
    x = start_state[1];
    y = start_state[2];
    vx = start_state[3];
    vy = start_state[4];
    s = start_state[5];
    d = start_state[6];
}

Vehicle::~Vehicle() {}

bool Vehicle::check_collision(int steps, double ref_s, double ref_d) {
    if (abs(d - ref_d) > 3.5) {
        return false;
    }
    double veh_speed = sqrt(vx * vx + vy * vy);
    double s_future = s + steps * .02 * veh_speed;
    if ((s_future > ref_s) && (s_future - ref_s < 30)) {
        return true;
    }
    return false;
}


Trajectory::Trajectory(vector<double> s_coeff,
                       vector<double> d_coeff,
                       double t) {
    this->t = t;
    this->s_coeff = s_coeff;
    this->d_coeff = d_coeff;
}

Trajectory::~Trajectory() {};

