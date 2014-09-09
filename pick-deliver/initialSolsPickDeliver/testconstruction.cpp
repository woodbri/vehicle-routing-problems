#include <deque>
#include <cmath>

#include "prob_pd.h"
#include "solution.h"
#include "vehicle.h"
#include "bucketn.h"
#include "testconstruction.h"


void TestConstruction::dumbConstruction() {
    Vehicle truck(depot,Q);
    fleet.clear();
        for (int i=0; i<ordersList.size(); i++) {
           truck.pushOrder(getOrder(i));
        }
   fleet.push_back(truck);
}

void TestConstruction::dumbConstructionAndBestMoveForward() {
    Vehicle truck(depot,Q);
    fleet.clear();
    sortOrdersbyDistReverse();
    int bestI;
    int bestJ;
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushOrder(getOrder(i));
        }
    truck.findBetterForward(bestI, bestJ);
    truck.move(bestI,bestJ);
    fleet.push_back(truck);
};

void TestConstruction::withSortedOrdersConstruction() {
    sortOrdersbyIdReverse();
    dumbConstruction();
    //sortOrdersbyDist();
};

void TestConstruction::dumbAndHillConstruction() {
    Vehicle truck(depot,Q);
    fleet.clear();
    sortOrdersbyDistReverse();
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushOrder(getOrder(i));
        }
    truck.hillClimbOpt();
    fleet.push_back(truck);
};

void TestConstruction::deliveryBeforePickupConstruction() {
    Vehicle truck(depot,Q);
    fleet.clear();
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushDelivery(getOrder(i));
           truck.pushPickup(getOrder(i));
        }
    fleet.push_back(truck);
};




