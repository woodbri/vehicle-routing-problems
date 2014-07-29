#ifndef ROUTE_H
#define ROUTE_H

#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

class Solution;     // forward reference

#include "Order.h"
#include "Problem.h"

class Route {
  public:
    int rid;

    Problem& P;

    std::vector<int> path;      // node ids along the path
    std::vector<int> orders;    // order ids associated with the nodes
//    std::vector<int> capacity;  // capacity after node is loaded
//    std::vector<double> pdist;  // distance at node max(arrival time, tw_open)
    bool updated;
    int D;      // duration
    int TWV;    // TW violations
    int CV;     // capacity violations
    double cost;

    // these are used by testPath()
    int tD;      // duration
    int tTWV;    // TW violations
    int tCV;     // capacity violations

    Route(Problem& p);

    // ~Route() {};

    Route &operator = (const Route &r) { P = r.P; return *this; };

    void update();

    double testPath(const std::vector<int>& tp);

    double getCost();

    void addOrder(const Order &o);

    bool insertOrder(int oid, bool mustBeValid);

    void removeOrder(const Order &o);

    void removeOrder(const int oid);

    int addPickup(const Order &o);

    void addDelivery(const Order &o);

    void hillClimbOpt();

    void dump();
};

#endif
