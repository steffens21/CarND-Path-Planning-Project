//
//  path_gen.h
//  
//
//  Created by Reinhard Steffens on 8/28/17.
//
//

#ifndef path_gen_h
#define path_gen_h
#include <cmath>
#include <algorithm>
#include <math.h>
#include "tools.h"
#include "spline.h"
#include <vector>
#include <stdio.h>

class Path {
public:
    Path();

    ~Path();

    vector<double> x_vals;
    vector<double> y_vals;

    void generate(double target_lane,
                  double target_vel,
                  double prev_x,
                  double curr_x,
                  double prev_y,
                  double curr_y,
                  double car_s,
                  double ref_yaw,
                  vector<double> maps_s,
                  vector<double> maps_x,
                  vector<double> maps_y);
};

#endif /* path_gen_h */
