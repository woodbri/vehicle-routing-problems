#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

#include "twpath.h"
#include "trashnode.h"


class Vehicle {
  private:
    Twpath<Trashnode> path;
    Trashnode backToDepot;
    Trashnode dumpsite;

    int maxcapacity;
    double cost;        // cost of the route

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost

  public:

    // structors

    Vehicle() {
        maxcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
    };

    Vehicle( Trashnode& _depot, Trashnode& _dump ) {
        maxcapacity  = _depot.getdemand();
        _depot.setdemand(0);
        _depot.setservice(0);
        backToDepot  = _depot;
        dumpsite = _dump;
        push_back( _depot );
        cost         = 0;
        w1 = w2 = w3 = 1.0;
    }

    // accessors
    Twpath<Trashnode> getvpath() const { return path; };
    std::deque<int> getpath();
    int size() const { return path.size(); };
    int getmaxcapacity() const { return maxcapacity; };
    int getTWV() const { return backToDepot.gettwvTot(); };
    int getCV() const { return backToDepot.getcvTot(); };
    int getcargo() const { return path.back().getcargo(); };
    double getduration() const { return backToDepot.gettotDist(); };
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

    void dump();
    void dumppath();

/* evaluation */

    bool feasable() const { return backToDepot.gettwvTot() == 0 and backToDepot.getcvTot() == 0; };
    bool hascv()const { return backToDepot.getcvTot() != 0; };
    bool hastwv()const { return backToDepot.gettwvTot() != 0; };

    void evalLast();

/* mutators */

    // these two do not work with autoeval
    // instead use Vehicle(depot, dump) constructor
    //void setdepot(Trashnode _depot) { backToDepot = _depot; };
    //void setdumpsite(Trashnode _dump) { dumpsite = _dump; };

    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    void push_back(Trashnode node);
    void push_front(Trashnode node);
    void insert(Trashnode node, int at);

/* algorithm specific */

    void doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4);
    void doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6);
    bool pathOptimize();
    bool pathTwoOpt();
    bool pathThreeOpt();

/* I really hate these shortcuts */

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

};


#endif

