#include <iostream>
#include "ptg.h"
#include <cmath>
#include <vector>
#include "cost.h"
#include "tools.h"
#include "veh.h"

using namespace std;


PTG::PTG(bool debug) {
    DEBUG = debug;
}

PTG::~PTG() {}

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
            float g_d = goals_d[j];

            vector<double> poly_coeff_s= JMT({my_s, my_speed, vehicle.state[2]},
                                             {g_s, target_speed, 0},
                                             goal_T);
            vector<double> poly_coeff_d = JMT({my_d, vehicle.state[4], vehicle.state[4]},
                                              {g_d, 0, 0},
                                              goal_T);

            Trajectory traj = Trajectory::Trajectory(poly_coeff_s,
                                                     poly_coeff_d,
                                                     goal_T);
            float this_cost = calculate_cost(traj);
            if (this_cost < min_cost) {
                min_cost = this_cost;
                best_s_coeff = poly_coeff_s;
                best_d_coeff = poly_coeff_d;
            }
            if (DEBUG) {
                std::cout << "Coeffs for s and d and cost" << std::endl;
                log_vector(poly_coeff_s);
                log_vector(poly_coeff_d);
                std::cout << this_cost << std::endl;
            }
        }
    }
    if (DEBUG) {
        std::cout << "Best coeffs for s and d" << std::endl;
        log_vector(best_s_coeff);
        log_vector(best_d_coeff);
    }

    for(int i = 1; i <= NBR_PRED_POINTS; i++) {
        double t = TIME_STEP * i;
        double poly_val_s = eval_traj(best_s_coeff, t);
        //double poly_val_s = my_s + t * target_speed;
        next_s_vals.push_back(poly_val_s);
        double poly_val_d = eval_traj(best_d_coeff, t);
        //double poly_val_d = my_d;
        next_d_vals.push_back(poly_val_d);
    }

    if(DEBUG) {
        std::cout << "next_s_vals " << std::endl;
        log_vector(next_s_vals);
    }
}


double PTG::calculate_cost(Trajectory traj){
    // TODO: tune weights after all cost functions are implemented
    double total_cost = 0.0;
    total_cost += 100 * collision_cost(traj, other_cars);
    std::cout << "collision_cost: " << collision_cost(traj, other_cars) << std::endl;
    total_cost += time_diff_cost(traj, other_cars);
    total_cost += s_diff_cost(traj, other_cars);
    total_cost += d_diff_cost(traj, other_cars);
    total_cost += buffer_cost(traj, other_cars);
    total_cost += stays_on_road_cost(traj, other_cars);
    total_cost += exceeds_speed_limit_cost(traj, other_cars);
    total_cost += efficiency_cost(traj, other_cars);
    total_cost += 10 * max_accel_cost(traj, other_cars);
    std::cout << "max_accel_cost: " << max_accel_cost(traj, other_cars) << std::endl;
    total_cost += 1 * total_accel_cost(traj, other_cars);
    std::cout << "total_accel_cost: " << total_accel_cost(traj, other_cars) << std::endl;
    total_cost += 50 * max_jerk_cost(traj, other_cars);
    std::cout << "max_jerk_cost: " << max_jerk_cost(traj, other_cars) << std::endl;
    total_cost += 5 * total_jerk_cost(traj, other_cars);
    std::cout << "total_jerk_cost: " << total_jerk_cost(traj, other_cars) << std::endl;

    return total_cost;
}

/*
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


*/
