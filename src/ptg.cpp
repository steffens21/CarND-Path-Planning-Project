#include <iostream>
#include "ptg.h"
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "cost.h"

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

void PTG::generatePath(double pos_x,
                       double pos_y,
                       double car_speed,
                       double car_accell,
                       double angle,
                       double end_path_s,
                       double end_path_d,
                       //double next_waypoints_x,
                       //double next_waypoints_y,
                       double next_waypoints_s,
                       //double next_waypoints_dx,
                       //double next_waypoints_dy,
                       int new_points_needed) {

    // TODO: Right now we let the car always drive in the middle lane.
    //       Eventually we have to use an algorithm to determine the best lane
    int desired_lane = 1;

    // the waypoints are not necessarily on the right lane.  You have to use the dx and dy to adapt.
    //double next_x_wayp = next_waypoints_x + next_waypoints_dx * (2 + desired_lane * 4);
    //double next_y_wayp = next_waypoints_y + next_waypoints_dy * (2 + desired_lane * 4);

    // TODO: limit speed, accelleration and jerk
    // TODO: don't collide with other cars

    // Use JMT
    double distance_to_point = next_waypoints_s - end_path_s;

    double time_to_point = distance_to_point / target_speed;

    vector<double> poly_coeff = JMT({end_path_s, car_speed, car_accell},
                                    {next_waypoints_s, target_speed, 0},
                                    time_to_point);
    if (DEBUG) {
        std::cout << "-------------------------" << std::endl;
        //std::cout << pos_x << " " << pos_y << " " << next_x_wayp << " " << next_y_wayp << std::endl;
        std::cout << "distance_to_point        " << distance_to_point << std::endl;
        std::cout << "time_to_point            " << time_to_point << std::endl;
        std::cout << "end_path_s               " << end_path_s << std::endl;
        std::cout << "next_waypoints_s         " << next_waypoints_s << std::endl;


        std::cout << "poly_coeff               "
        << poly_coeff[0]
        << " " << poly_coeff[1]
            << " " << poly_coeff[2]
            << " " << poly_coeff[3]
            << " " << poly_coeff[4]
            << poly_coeff[5] << std::endl;
    }

    for(int i = 1; i <= new_points_needed; i++) {
        double t = 0.02 * i;
        //double poly_val = poly_eval(poly_coeff, t); // TODO: use this
        // brute force, no accell or jerk minimizing
        double poly_val = end_path_s + t * distance_to_point / time_to_point;
        if (poly_val > next_waypoints_s) {
            continue;
        }
        next_s_vals.push_back(poly_val);
    }

    if (DEBUG) {
        for(int i=0;i<next_s_vals.size();i++) {
            std::cout << next_s_vals[i] << " ";
        }
        std::cout << std::endl;

        for(int i=1;i<next_s_vals.size();i++) {
            std::cout << (next_s_vals[i] - next_s_vals[i-1]) / 0.02 << " ";
        }
        std::cout << std::endl;
    }
}


double PTG::calculate_cost(vector<double> traj,
                           int target_vehicle,
                           double delta,
                           double goal_T,
                           vector<double> predictions){
    // TODO: probably don't need predictions.  Just use ptg.other_cars
    // TODO: set-up cost_functions_with_weigths
    double total_cost = 0.0;
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
