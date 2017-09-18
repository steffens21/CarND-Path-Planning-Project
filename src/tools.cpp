#include <math.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "tools.h"
#include "veh.h"

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
    int next_wp = NextWaypoint(x, y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double wayp_yaw = atan2(n_y, n_x);
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d, theta - wayp_yaw};

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

double collision_dist(double ref_s,
                      double ref_d,
                      double ref_x,
                      double ref_y,
                      vector<Vehicle> other_cars,
                      int steps) {
    double min_dist = 999.0;

    for(int i=0; i<other_cars.size(); i++) {
        Vehicle veh = other_cars[i];
        if (abs(veh.d - ref_d) > 2.2) {
            continue;
        }
        double veh_speed = sqrt(veh.vx * veh.vx + veh.vy * veh.vy);
        double s_future = veh.s + steps * .02 * veh_speed;
        double veh_x_fut = veh.x + .02 * steps * veh.vx;
        double veh_y_fut = veh.y + .02 * steps * veh.vy;

        double dist = distance(ref_x, ref_y, veh_x_fut, veh_y_fut);

        if ((s_future > ref_s - 8) && (dist < min_dist)) {
            min_dist = dist;
        }
    }
    return min_dist;
}

vector<double> getTargetSpeedAndLane(double ref_s,
                                     double ref_d,
                                     double ref_x,
                                     double ref_y,
                                     double d_diff,
                                     double ref_speed,
                                     vector<Vehicle> other_cars,
                                     int path_size,
                                     bool DEBUG) {

    double dist_tolerance = 40.0;

    int ref_lane = 2;
    if (ref_d < 4) {
        ref_lane = 0;
    }
    else if (ref_d < 8) {
        ref_lane = 1;
    }

    // FSM to decide target lane

    double target_lane = ref_lane;
    bool slower = false;
    double min_col_dist;

    double col_dist[3];
    col_dist[0] = collision_dist(ref_s,
                                 2,
                                 ref_x,
                                 ref_y,
                                 other_cars,
                                 path_size);
    col_dist[1] = collision_dist(ref_s,
                                 6,
                                 ref_x,
                                 ref_y,
                                 other_cars,
                                 path_size);
    col_dist[2] = collision_dist(ref_s,
                                 10,
                                 ref_x,
                                 ref_y,
                                 other_cars,
                                 path_size);

    if(DEBUG) {
        std::cout << "Lane: Dist" << std::endl;
        std::cout << "   0: " << col_dist[0] << std::endl;
        std::cout << "   1: " << col_dist[1] << std::endl;
        std::cout << "   2: " << col_dist[2] << std::endl;
    }


    min_col_dist = col_dist[ref_lane];
    if (col_dist[ref_lane] > dist_tolerance) {
        target_lane = ref_lane;
    }
    else {
        slower = true;
        if (ref_lane > 0) {
            if (ref_lane == 2) {
                if(col_dist[1] > dist_tolerance + 5) {
                    target_lane = 1;
                    slower = false;
                }
            }
            else {
                double best_dist;
                int cand_lane;
                if (col_dist[0] > col_dist[2]) {
                    best_dist = col_dist[0];
                    cand_lane = 0;
                }
                else {
                    best_dist = col_dist[2];
                    cand_lane = 2;
                }
                if(best_dist > dist_tolerance + 5) {
                    target_lane = cand_lane;
                    slower = false;
                }
            }
        }
        else {
            if (col_dist[1] > dist_tolerance + 5) {
                target_lane = 1;
                slower = false;
            }
        }
    }

    // don't change lanes when you are slow
    if (ref_speed < 20) {
        target_lane = ref_lane;
    }

    // prefer middle lane if there is enough space
    if (col_dist[1] > dist_tolerance + 10) {
        target_lane = 1;
        min_col_dist = col_dist[1];
        slower = false;
    }

    // Adapt speed
    double target_vel = ref_speed;
    
    if (slower) {
        if (min_col_dist < 5.0) {
            target_vel -= 0.15 * (50 - path_size);
        }
        else if (min_col_dist < 15) {
            target_vel -= 0.05 * (50 - path_size);
        }
        else {
            double steps_to_col = (min_col_dist / ref_speed) / 0.02;
            double col_steps_portion = (50 - path_size) / steps_to_col;
            target_vel -= 0.025 * col_steps_portion;
        }
        target_vel = max(0.0, target_vel);
        if(DEBUG) {
            std::cout << " ** break" << std::endl;
        }
    }
    else if (target_vel < 50) {
        target_vel += 0.185 * (50 - path_size);
        target_vel = min(49.8, target_vel);
        if (DEBUG) {
            std::cout << " ** accell" << std::endl;
        }
    }

    // slow accell at beginning
    if (ref_speed < 2.0) {
        target_vel = 3.0;
    }
    
    return {target_vel, target_lane};
}



