#ifndef NODE_H
#define NODE_H

#include <iostream>

class Node {
  public:
    int nid;        // node number (0 = depot
    double x;       // x location
    double y;       // y location
    int demand;     // capacity demand
    int tw_open;    // earliest window time
    int tw_close;   // latest window time
    int service;    // service time
    int pid;        // pickup (id of sibling)
    int did;        // delivery (id of sibling)

    // Node() {};
    // ~Node() {};

    void dump();
};

#endif
