#include <iostream>
#include "ptg.h"
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


PTG::PTG() {}

PTG::~PTG() {}

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
  for(int i = 0; i < C.size(); i++)
    {
      result.push_back(C.data()[i]);
    }

  return result;
}

void PTG::generatePath(float pos_x, float pos_y, float angle, vector<double> previous_path_x, vector<double> previous_path_y, double next_waypoints_x, double next_waypoints_y, double next_waypoints_dx, double next_waypoints_dy)
{
  int path_size = previous_path_x.size();

  for(int i = max(path_size - 5, 0); i < path_size; i++){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  int desired_lane = 1;

  // the waypoints are not necessarily on the right lane.  You have to use the dx and dy to adapt.
  double next_x_wayp = next_waypoints_x + next_waypoints_dx * (2 + desired_lane * 4);
  double next_y_wayp = next_waypoints_y + next_waypoints_dy * (2 + desired_lane * 4);

  // TODO: limit speed, accelleration and jerk
  // TODO: don't collide with other cars
  // TODO: use spline for smoothing

  double dist_inc = 0.1;
  double new_x = pos_x;
  double new_y = pos_y;
  std::cout << pos_x << " " << pos_y << " " << next_x_wayp << " " << next_y_wayp << std::endl;

  //double sq_dist_next_wayp = (next_x_wayp - pos_x)**2 + (next_y_wayp - pos_y)**2;

  double x_fact = (next_x_wayp - pos_x) / float(50 - path_size);
  double y_fact = (next_y_wayp - pos_y) / float(50 - path_size);
  double sq_fact = x_fact*x_fact + y_fact*y_fact;
  x_fact = x_fact / sq_fact * dist_inc;
  y_fact = y_fact / sq_fact * dist_inc;
  std::cout << x_fact << " " << y_fact << std::endl;
  for(int i = 0; i < 50-path_size; i++) {
    //pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
    //pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    new_x += x_fact;
    new_y += y_fact;
    next_x_vals.push_back(new_x);
    next_y_vals.push_back(new_y);
  }
  /*
    for(int i=0;i<next_x_vals.size();i++) {
    std::cout << next_x_vals[i] << " ";
    }
    std::cout << std::endl;
    for(int i=0;i<next_y_vals.size();i++) {
    std::cout << next_y_vals[i] << " ";
    }
    std::cout << std::endl;
  */
  
}



/*
def time_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
    """
    _, _, t = traj
    return logistic(float(abs(t-T)) / T)

def s_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
    """
    s, _, T = traj
    target = predictions[target_vehicle].state_in(T)
    target = list(np.array(target) + np.array(delta))
    s_targ = target[:3]
    S = [f(T) for f in get_f_and_N_derivatives(s, 2)]
    cost = 0
    for actual, expected, sigma in zip(S, s_targ, SIGMA_S):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

def d_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
    """
    _, d_coeffs, T = traj
    
    d_dot_coeffs = differentiate(d_coeffs)
    d_ddot_coeffs = differentiate(d_dot_coeffs)

    d = to_equation(d_coeffs)
    d_dot = to_equation(d_dot_coeffs)
    d_ddot = to_equation(d_ddot_coeffs)

    D = [d(T), d_dot(T), d_ddot(T)]
    
    target = predictions[target_vehicle].state_in(T)
    target = list(np.array(target) + np.array(delta))
    d_targ = target[3:]
    cost = 0
    for actual, expected, sigma in zip(D, d_targ, SIGMA_D):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

def collision_cost(traj, target_vehicle, delta, T, predictions):
    """
    Binary cost function which penalizes collisions.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    if nearest < 2*VEHICLE_RADIUS: return 1.0
    else : return 0.0

def buffer_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes getting close to other vehicles.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    return logistic(2*VEHICLE_RADIUS / nearest)
    
def stays_on_road_cost(traj, target_vehicle, delta, T, predictions):
    pass

def exceeds_speed_limit_cost(traj, target_vehicle, delta, T, predictions):
    pass

def efficiency_cost(traj, target_vehicle, delta, T, predictions):
    """
    Rewards high average speeds.
    """
    s, _, t = traj
    s = to_equation(s)
    avg_v = float(s(t)) / t
    targ_s, _, _, _, _, _ = predictions[target_vehicle].state_in(t)
    targ_v = float(targ_s) / t
    return logistic(2*float(targ_v - avg_v) / avg_v)

def max_accel_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        acc = a(t)
        total_acc += abs(acc*dt)
    acc_per_second = total_acc / T
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )
    
def total_accel_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(T)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0
    

def max_jerk_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
    jerk = to_equation(jerk)
    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > MAX_JERK: return 1
    else: return 0

def total_jerk_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = to_equation(differentiate(s_d_dot))
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )


class Vehicle(object):
    """
    Helper class. Non-ego vehicles move w/ constant acceleration
    """
    def __init__(self, start):
        self.start_state = start
    
    def state_in(self, t):
        s = self.start_state[:3]
        d = self.start_state[3:]
        state = [
            s[0] + (s[1] * t) + s[2] * t**2 / 2.0,
            s[1] + s[2] * t,
            s[2],
            d[0] + (d[1] * t) + d[2] * t**2 / 2.0,
            d[1] + d[2] * t,
            d[2],
        ]
        return state

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
