#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "spline.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x,
                    double y,
                    vector<double> maps_x,
                    vector<double> maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x,
                 double y,
                 double theta,
                 vector<double> maps_x,
                 vector<double> maps_y) {

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x,
                         double y,
                         double theta,
                         vector<double> maps_x,
                         vector<double> maps_y) {
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY_mod(double s,
                         double d,
                         vector<double> maps_s,
                         vector<double> maps_x,
                         vector<double> maps_y) {

    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    if (prev_wp == -1) {
        prev_wp = 0;
    }

    // interpolate between wp3 and prev_wp
    int wp2 = (prev_wp + 1) % maps_x.size();
    int wp3 = (wp2 + 1) % maps_x.size();

    tk::spline spline_s_x;
    tk::spline spline_s_y;
    // TODO: think about adding dx values here
    vector<double> maps_x_sel = {maps_x[prev_wp], maps_x[wp2], maps_x[wp3]};
    vector<double> maps_y_sel = {maps_y[prev_wp], maps_y[wp2], maps_y[wp3]};
    vector<double> maps_s_sel = {maps_s[prev_wp], maps_s[wp2], maps_s[wp3]};

    vector<double> maps_x_mod;
    vector<double> maps_y_mod;
    vector<double> maps_s_mod;


    spline_s_x.set_points(maps_s_sel,
                          maps_x_sel);
    spline_s_y.set_points(maps_s_sel,
                          maps_y_sel);

    int samples = 40;
    float step_s = abs(maps_s_sel[-1] - maps_s_sel[0]) / float(samples);
    for (int i = 0; i < samples; i++) {
        float s_val = maps_s_sel[0] + i * step_s;
        maps_x_mod.push_back(spline_s_x(s_val));
        maps_y_mod.push_back(spline_s_y(s_val));
        maps_s_mod.push_back(s_val);
    }

    // now perform original getXY calculation with refined waypoints
    int prev_wp_mod = -1;
    while(s > maps_s_mod[prev_wp_mod+1]
          && (prev_wp_mod < (int)(maps_s_mod.size()-1))) {
        prev_wp_mod++;
    }
    // the modulus should never be applied
    int wp2_mod = (prev_wp_mod + 1) % maps_x_mod.size();

    double heading = atan2((maps_y_mod[wp2_mod]-maps_y_mod[prev_wp_mod]),
                           (maps_x_mod[wp2_mod]-maps_x_mod[prev_wp_mod]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s_mod[prev_wp_mod]);

    double seg_x = maps_x_mod[prev_wp_mod] + seg_s * cos(heading);
    double seg_y = maps_y_mod[prev_wp_mod] + seg_s * sin(heading);

    double perp_heading = heading - pi()/2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x,y};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s,
                     double d,
                     vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y) {
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                           (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading-pi() / 2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}

// Transform vehicle state of form [x, y, vx, vy, s, d]
// to state of form [s, s_dot, s_dotdot, d, d_dot, d_dotdot]
// Obviously the accellerations have to be assumed to be 0
vector<double> transVehState(double x,
                             double y,
                             double vx,
                             double vy,
                             double s,
                             double d) {
    double delta_theta = atan2(vy, vx);
    double mag_v = sqrt(pow(vx,2) + pow(vy,2));
    double v_s = mag_v * cos(delta_theta);
    double v_d = mag_v * sin(delta_theta);

    vector<double> restult_state;
    restult_state.push_back(s);
    restult_state.push_back(v_s);
    restult_state.push_back(0.0);
    restult_state.push_back(d);
    restult_state.push_back(v_d);
    restult_state.push_back(0.0);

    return restult_state;
}
