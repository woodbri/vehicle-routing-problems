/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
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

