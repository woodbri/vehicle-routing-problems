#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "Node.h"
#include "Order.h"

const double w1 = 1.0;  // route duration weighting
const double w2 = 1000.0;  // total number of time violations weight
const double w3 = 1.0;  // total number of capacity violations weight

class Problem {
  public:
    int K;      // number of vehicles
    int Q;      // capacity
    int DepotClose;
    double atwl;
    std::vector<Node> N;    // vector of nodes
    std::vector<Order> O;   // vector of orders

    // variables for plotting
    double extents[4];

    // Problem() {};
    // ~Problem() {};

    void loadProblem(char *infile);

    unsigned int getNodeCount();

    unsigned int getOrderCount();

    double distance(int n1, int n2) const;

    void makeOrders();

    void dump();

    void calcAvgTWLen();
};

#endif
