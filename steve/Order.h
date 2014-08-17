#ifndef ORDER_H
#define ORDER_H

#include <iostream>

class Order {
  public:
    int oid;        // order id
    int pid;        // pickup node id
    int did;        // delivery node id
    double dist;    // distance from depot to pickup location
    double dist2;   // distance from delivery to depot

    // Order() {};
    // ~Order() {};

    void dump();
};

#endif
