#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

#include "twpath.h"
#include "trashnode.h"

// TODO: change curcapacity to curload
//       add getcurload() and change getcurcapacity(0 to return
//       getmaxcapacity()-curload

class Vehicle : public Twpath<Trashnode> {
  private:
    int curcapacity;    // current USED capacity of the vehicle
    double duration;    // duration of the route
    double cost;        // cost of the route
    int TWV;            // number of time window violations
    int CV;             // number of capacity violations

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost

  public:

    // structors

    Vehicle() {
        //maxcapacity = 0;
        curcapacity = 0;
        duration    = 0;
        cost        = 0;
        TWV         = 0;
        CV          = 0;
        w1 = w2 = w3 = 1.0;
    };

    // accessors
    std::deque<int> getpath();
    int getmaxcapacity() {              //// should be const
        return getdepot().getdemand();
    };
    int getTWV() const { return TWV; };
    int getCV() const { return CV; };
    int getcurcapacity() const { return curcapacity; };
    double getduration() const { return duration; };
    double getcost() const { return cost; };
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };

    // these should be const
    double distancetodepot(int i) { return path[i].distance(getdepot()); };
    double distancetodump(int i) { return path[i].distance(getdumpsite()); };

    void dump();
    void dumppath();

    // mutators
    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    void evaluate();

};


#endif

