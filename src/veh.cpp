#include <iostream>
#include "veh.h"
#include <vector>

using namespace std;

Vehicle::Vehicle(int id, vector<double> start_state) {
    this->id = id;
    state = start_state;
}

Vehicle::~Vehicle() {}

vector<double> Vehicle::state_in(double t) {
    vector<double> new_state;
    new_state.push_back(state[0] + (state[1] * t) + state[2] * t * t / 2.0);
    new_state.push_back(state[1] + state[2] * t);
    new_state.push_back(state[2]);
    new_state.push_back(state[3] + (state[4] * t) + state[5] * t * t / 2.0);
    new_state.push_back(state[4] + state[5] * t);
    new_state.push_back(state[5]);
    return new_state;
}

void Vehicle::log_state() {
    std::cout << state[0] << " "
	      << state[1] << " "
	      << state[2] << " "
              << state[3] << " "
              << state[4] << " "
              << state[5] << " " << std::endl;
}

Trajectory::Trajectory(vector<double> s_coeff,
                       vector<double> d_coeff,
                       double t) {
    this->t = t;
    this->s_coeff = s_coeff;
    this->d_coeff = d_coeff;
}

Trajectory::~Trajectory() {};

