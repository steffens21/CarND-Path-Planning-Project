#ifndef VEH_H_
#define VEH_H_
#include <vector>
using namespace std;

class Vehicle {
    public:
        /**
         * Constructor.
         */
        Vehicle(vector<double> start_state);

        /**
         * Destructor.
         */
        virtual ~Vehicle();

        /**
         * Vehicle state
         */
        int id;
        float x;
        float y;
        float vx;
        float vy;
        float s;
        float d;

    bool check_collision(int steps, double ref_s, double ref_d);
};

class Trajectory {
public:
    Trajectory(vector<double> s_coeff, vector<double> d_coeff, double t);

    virtual ~Trajectory();

    vector<double> s_coeff;
    vector<double> d_coeff;
    double t;
};


#endif /* VEH_H_ */
