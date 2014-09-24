#ifndef TESTCONSTRUCTION_H
#define TESTCONSTRUCTION_H


#include "solution.h"
//#include "orders.h"

//const double EPSILON = 0.001;

class TestConstruction: public Solution {

  public:
    TestConstruction(const Solution& P):Solution(P){ }; 

    void dumbConstruction();
    void dumbConstructionAndBestMoveForward();
    void withSortedOrdersConstruction();
    void dumbAndHillConstruction();
    void deliveryBeforePickupConstruction();

};
#endif

