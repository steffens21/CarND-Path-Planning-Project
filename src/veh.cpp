#include <iostream>
#include <cmath>
#include <algorithm>
#include <math.h>
#include "veh.h"
#include <vector>

using namespace std;

Vehicle::Vehicle(float _x,
                 float _y,
                 float _vx,
                 float _vy,
                 float _s,
                 float _d) {
    x = _x;
    y = _y;
    vx = _vx;
    vy = _vy;
    s = _s;
    d = _d;
}

Vehicle::~Vehicle() {}


