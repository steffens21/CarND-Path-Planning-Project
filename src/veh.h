#ifndef VEH_H_
#define VEH_H_
#include <vector>
using namespace std;

class Vehicle {
   public:
      /**
       * Constructor.
       */
      Vehicle(int id, vector<double> start_state);

      /**
       * Destructor.
       */
      virtual ~Vehicle();

      /**
       * Vehicle id
       */
      int id;

      /**
       * Vehicle state
       */
      vector<double> state;

      /**
       * Return state of vehicle in t sec
       */
      vector<double> state_in(double t);

      void log_state();
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
