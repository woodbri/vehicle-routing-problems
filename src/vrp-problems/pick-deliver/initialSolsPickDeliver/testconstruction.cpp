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
#include <deque>
#include <cmath>

#include "prob_pd.h"
#include "solution.h"
#include "vehicle.h"
#include "bucketn.h"
#include "testconstruction.h"



//I want all this tests to generate valid solutions



// PUSH order construction
void TestConstruction::dumbConstruction() {
    Orders orders=ordersList;
    Orders unassigned;
    Order order;
    fleet.clear();
    while (not orders.empty()) {
       Vehicle truck(depot,Q);
       while (not orders.empty()) {
          order=orders[0];
          orders.erase(0);
          if (truck.pushOrder(order) )  continue ;
          else  unassigned.push_back(order);
       }
       orders.join(unassigned);
       unassigned.clear();
       fleet.push_back(truck);
    }
}

void TestConstruction::dumbConstructionAndBestMoveForward() {
/*    Vehicle truck(depot,Q);
    fleet.clear();
    sortOrdersbyDistReverse();
    int bestI;
    int bestJ;
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushOrder(getOrder(i));
        }
    truck.findBetterForward(bestI, bestJ);
    truck.e_move(bestI,bestJ);
    fleet.push_back(truck);
*/
};

void TestConstruction::withSortedOrdersConstruction() {
/*    sortOrdersbyIdReverse();
    dumbConstruction();
    //sortOrdersbyDist();
*/
};

void TestConstruction::dumbAndHillConstruction() {
/*    Vehicle truck(depot,Q);
    fleet.clear();
    sortOrdersbyDistReverse();
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushOrder(getOrder(i));
        }
    truck.hillClimbOpt();
    fleet.push_back(truck);
*/
};

void TestConstruction::deliveryBeforePickupConstruction() {
/*    Vehicle truck(depot,Q);
    fleet.clear();
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushDelivery(getOrder(i));
           truck.pushPickup(getOrder(i));
        }
    fleet.push_back(truck);
*/
};




