//
//  path_gen.cpp
//  
//
//  Created by Reinhard Steffens on 8/28/17.
//
//

#include <cmath>
#include <algorithm>
#include <math.h>
#include "path_gen.h"
#include "spline.h"
#include "tools.h"
#include <vector>

using namespace std;


Path::Path() {};

Path::~Path() {};


void Path::generate(double target_lane,
                    double target_vel,
                    double prev_x,
                    double curr_x,
                    double prev_y,
                    double curr_y,
                    double car_s,
                    double ref_yaw,
                    vector<double> maps_s,
                    vector<double> maps_x,
                    vector<double> maps_y) {

    vector<double> ptsx;
    vector<double> ptsy;

    ptsx.push_back(prev_x);
    ptsx.push_back(curr_x);
    ptsy.push_back(prev_y);
    ptsy.push_back(curr_y);


    // In Frenet add somewhat evenly spaced points ahead of the
    // starting reference
    vector<double> next_wp0 = getXY(car_s + 50,
                                    (2 + 4 * target_lane),
                                    maps_s,
                                    maps_x,
                                    maps_y);
    vector<double> next_wp1 = getXY(car_s + 110,
                                    (2 + 4 * target_lane),
                                    maps_s,
                                    maps_x,
                                    maps_y);
    vector<double> next_wp2 = getXY(car_s + 175,
                                    (2 + 4 * target_lane),
                                    maps_s,
                                    maps_x,
                                    maps_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i=0; i<ptsx.size(); i++) {
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - curr_x;
        double shift_y = ptsy[i] - curr_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw)
                   - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw)
                   + shift_y * cos(0 - ref_yaw));
    }

    // create a spline
    tk::spline sp;

    // set (x,y) points to the spline
    sp.set_points(ptsx, ptsy);


    // Calculate how to break up spline points so that we travel
    // at our desired reference velocity
    double target_x = 30.0;
    double target_y = sp(target_x);
    double target_dist = distance(target_x, target_y, 0, 0);

    double x_add_on = 0.0;

    // Fill path points
    for (int i=0; i<=50; i++) {
        double N = target_dist / (.02 * target_vel / 2.24);
        double x_point = x_add_on + target_x / N;
        double y_point = sp(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal to offset earlier rotation
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += curr_x;
        y_point += curr_y;

        x_vals.push_back(x_point);
        y_vals.push_back(y_point);
    }
}

double Path::path_cost() {
    double total_cost = 0.0;

    return total_cost;
}
