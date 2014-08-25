#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

#include "twpath.h"
#include "trashnode.h"

// TODO: change curcapacity to curload
//       add getcurload() and change getcurcapacity(0 to return
//       getmaxcapacity()-curload

class Vehicle {
  private:
    Twpath<Trashnode> path;
    Trashnode backToDepot;
    Trashnode dumpsite;

    int maxcapacity;
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

    Vehicle( Trashnode& _depot ) {
        maxcapacity  = _depot.getdemand();
        _depot.setdemand(0);
        backToDepot  = _depot;
        push_back( _depot );
        curcapacity  = 0;
        cost         = 0;
        w1 = w2 = w3 = 1.0;
    }

    // accessors
    std::deque<int> getpath();
    int size() const { return path.size(); };
    int getmaxcapacity() const { return maxcapacity; };
    int getTWV() const { return TWV; };
    int getCV() const { return CV; };
    int getcurcapacity() const { return curcapacity; };
    double getduration() const { return duration; };
    double getcost() const { return cost; };
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };
    Trashnode getdepot() const { return backToDepot; };
    Trashnode& getdepot() { return backToDepot; };
    Trashnode getdumpsite() const { return dumpsite; };
    Trashnode& getdumpsite() { return dumpsite; };

    double distancetodepot(int i) const { return path[i].distance(getdepot()); };
    double distancetodump(int i) const { return path[i].distance(getdumpsite()); };

    Trashnode operator[](int i) const { return path[i]; };

    void push_back(Trashnode node);
    void push_front(Trashnode node);
    void insert(Trashnode node, int at);

    // I really hate these shortcuts
    int getnid(int i) const { return path[i].getnid(); };
    double getx(const int i) const { path[i].getx(); };
    double gety(const int i) const { path[i].gety(); };
    bool hasdemand(int i) const { return path[i].hasdemand(); };
    bool hassupply(int i) const { return path[i].hassupply(); };
    bool hasnogoods(int i) const { return path[i].hasnogoods(); };
    bool earlyarrival(int i, const double D) const { return path[i].earlyarrival(D); };
    bool latearrival(int i, const double D) const { return path[i].latearrival(D); };
    bool ontime(int i, const double D) const { return not earlyarrival(i, D) and not latearrival(i, D); };
    bool isdump(int i) const { return path[i].isdump(); };
    bool ispickup(int i) const { return path[i].ispickup(); };
    bool isdepot(int i) const { return path[i].hasnogoods(); };

    void dump();
    void dumppath();

    // mutators
    void setdepot(Trashnode _depot) { backToDepot = _depot; };
    void setdumpsite(Trashnode _dump) { dumpsite = _dump; };
    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    void evaluate();
    void evaluate(int from);
    void evalLast();

};


#endif

