#ifndef VEH_H_
#define VEH_H_
#include <vector>
using namespace std;

class Vehicle {
    public:
    /**
    * Constructor.
    */
    Vehicle(float x,
            float y,
            float vx,
            float vy,
            float s,
            float d);

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

};


#endif /* VEH_H_ */
