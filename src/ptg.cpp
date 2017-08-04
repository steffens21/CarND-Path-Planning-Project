#include <iostream>
#include "ptg.h"
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "cost.h"
#include "tools.h"
#include "veh.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


PTG::PTG(bool debug) {
    DEBUG = debug;
}

PTG::~PTG() {}

// eval poly of degree 5
double PTG::poly_eval(vector<double> a, double x) {
    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x2 * x2;
    double x5 = x3 * x2;
    return a[0] + a[1] * x + a[2] * x2 + a[3] * x3 + a[4] * x4 + a[5] * x5;
}

//TODO: move to tools.cpp
vector<double> PTG::JMT(vector< double> start, vector <double> end, double T) {
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
     */

    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
        3*T*T, 4*T*T*T,5*T*T*T*T,
        6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
        end[1]-(start[1]+start[2]*T),
        end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++) {
        result.push_back(C.data()[i]);
    }

    return result;
}

void PTG::generatePath() {

    if(DEBUG) {
        vehicle.log_state();
    }

    int NBR_PRED_POINTS = 50;
    float TIME_STEP = 0.02;

    float goal_T = NBR_PRED_POINTS * TIME_STEP;

    float my_s = vehicle.state[0];
    float my_speed = vehicle.state[1];
    float my_d = vehicle.state[3];

    float goal_s = my_s + target_speed * goal_T;
    float goal_d = 6; //my_d; // TODO: don't always stay in lane

    vector<double> goals_s = {goal_s - 2, goal_s, goal_s +2 };
    vector<double> goals_d = {10, 6, 2};
    float min_cost = 99999.9;
    vector<double> best_s_coeff;
    vector<double> best_d_coeff;

    for (int i=0; i<goals_s.size(); i++) {
        for (int j=0; j<goals_d.size(); j++) {
            float g_s = goals_s[i];
            float g_d = goals_d[i];

            vector<double> poly_coeff_s= JMT({my_s, my_speed, vehicle.state[2]},
                                             {goal_s, target_speed, 0},
                                             goal_T);
            log_vector(poly_coeff_s);
            vector<double> poly_coeff_d = JMT({my_d, vehicle.state[4], vehicle.state[4]},
                                              {goal_d, 0, 0},
                                              goal_T);
            log_vector(poly_coeff_d);

            Trajectory traj = Trajectory::Trajectory(poly_coeff_s,
                                                     poly_coeff_d,
                                                     goal_T);
            float this_cost = calculate_cost(traj);
            if (this_cost < min_cost) {
                min_cost = this_cost;
                best_s_coeff = poly_coeff_s;
                best_d_coeff = poly_coeff_d;
            }
        }
    }


    for(int i = 1; i <= NBR_PRED_POINTS; i++) {
        double t = TIME_STEP * i;
        double poly_val_s = poly_eval(best_s_coeff, t);
        //double poly_val_s = my_s + t * target_speed;
        next_s_vals.push_back(poly_val_s);
        double poly_val_d = poly_eval(best_d_coeff, t);
        //double poly_val_d = my_d;
        next_d_vals.push_back(poly_val_d);
    }

    if(DEBUG) {
        std::cout << "next_s_vals " << std::endl;
        log_vector(next_s_vals);
    }
}


double PTG::calculate_cost(Trajectory traj){
    // TODO: set-up cost_functions_with_weigths
    double total_cost = 0.0;
    total_cost += collision_cost(traj, other_cars);
    // TODO: loop over all cost functions,
    //       - add weight * cost to total_cost
    return total_cost;
}

/*
def logistic(x):
    """
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """
    return 2.0 / (1 + exp(-x)) - 1.0

def to_equation(coefficients):
    """
    Takes the coefficients of a polynomial and creates a function of
    time from them.
    """
    def f(t):
        total = 0.0
        for i, c in enumerate(coefficients): 
            total += c * t ** i
        return total
    return f

def differentiate(coefficients):
    """
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
    """
    new_cos = []
    for deg, prev_co in enumerate(coefficients[1:]):
        new_cos.append((deg+1) * prev_co)
    return new_cos

def nearest_approach_to_any_vehicle(traj, vehicles):
    """
    Calculates the closest distance to any vehicle during a trajectory.
    """
    closest = 999999
    for v in vehicles.values():
        d = nearest_approach(traj,v)
        if d < closest:
            closest = d
    return closest

def nearest_approach(traj, vehicle):
    closest = 999999
    s_,d_,T = traj
    s = to_equation(s_)
    d = to_equation(d_)
    for i in range(100):
        t = float(i) / 100 * T
        cur_s = s(t)
        cur_d = d(t)
        targ_s, _, _, targ_d, _, _ = vehicle.state_in(t)
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest

def show_trajectory(s_coeffs, d_coeffs, T, vehicle=None):
    s = to_equation(s_coeffs)
    d = to_equation(d_coeffs)
    X = []
    Y = []
    if vehicle:
        X2 = []
        Y2 = []
    t = 0
    while t <= T+0.01:
        X.append(s(t))
        Y.append(d(t))
        if vehicle:
            s_, _, _, d_, _, _ = vehicle.state_in(t)
            X2.append(s_)
            Y2.append(d_)
        t += 0.25
    plt.scatter(X,Y,color="blue")
    if vehicle:
        plt.scatter(X2, Y2,color="red")
    plt.show()

def get_f_and_N_derivatives(coeffs, N=3):
    functions = [to_equation(coeffs)]
    for i in range(N):
        coeffs = differentiate(coeffs)
        functions.append(to_equation(coeffs))
    return functions

# TODO - tweak weights to existing cost functions
WEIGHTED_COST_FUNCTIONS = [
    (time_diff_cost,    1),
    (s_diff_cost,       1),
    (d_diff_cost,       1),
    (efficiency_cost,   1),
    (max_jerk_cost,     1),
    (total_jerk_cost,   1),
    (collision_cost,    1),
    (buffer_cost,       1),
    (max_accel_cost,    1),
    (total_accel_cost,  1),
]

def PTG(start_s, start_d, target_vehicle, delta, T, predictions):
    """
    Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).

    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that 
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at 
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal 
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a 
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method 
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/ 
     this trajectory.
    """
    target = predictions[target_vehicle]
    # generate alternative goals
    all_goals = []
    timestep = 0.5
    t = T - 4 * timestep
    while t <= T + 4 * timestep:
        target_state = np.array(target.state_in(t)) + np.array(delta)
        goal_s = target_state[:3]
        goal_d = target_state[3:]
        goals = [(goal_s, goal_d, t)]
        for _ in range(N_SAMPLES):
            perturbed = perturb_goal(goal_s, goal_d)
            goals.append((perturbed[0], perturbed[1], t))
        all_goals += goals
        t += timestep

    # find best trajectory
    trajectories = []
    for goal in all_goals:
        s_goal, d_goal, t = goal
        s_coefficients = JMT(start_s, s_goal, t)
        d_coefficients = JMT(start_d, d_goal, t)
        trajectories.append(tuple([s_coefficients, d_coefficients, t]))

    best = min(trajectories, key=lambda tr: calculate_cost(tr, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS))
    calculate_cost(best, target_vehicle, delta, T, predictions, WEIGHTED_COST_FUNCTIONS, verbose=True)
    return best


def calculate_cost(trajectory, target_vehicle, delta, goal_t, predictions, cost_functions_with_weights, verbose=False):
    cost = 0
    for cf, weight in cost_functions_with_weights:
        new_cost = weight * cf(trajectory, target_vehicle, delta, goal_t, predictions)
        cost += new_cost
        if verbose:
            print "cost for {} is \t {}".format(cf.func_name, new_cost)
    return cost
*/
