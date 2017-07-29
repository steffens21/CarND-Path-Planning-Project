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
  vector<double> state;

  /**
   * Return state of vehicle in t sec
   */
  vector<double> state_in(double t);

  void log_state();
};

#endif /* VEH_H_ */
