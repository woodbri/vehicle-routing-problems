#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

#include "node.h"


class Problem {
  private:

  public:
    std::vector<Node> Nodes;    // vector of all nodes
    std::vector<int> Vehicles;  // vector of all vehicle nids
    std::vector<int> Dumps;     // vector of all dump nids
    std::vector<int> Pickups;   // vector of all pickup point nids

    // variables for plotting
    double extents[4];

    void loadProblem(char *infile);

    unsigned int getNodeCount();
    unsigned int getVehicleCount();
    unsigned int getDumpCount();
    unsigned int getPickupCount();

    double distance(int n1, int n2) const;
    void setNodeDistances(Node& n);

    void dumpVehicles();
    void dumpDumps();
    void dumpPickups();
    void dump();

};

#endif
