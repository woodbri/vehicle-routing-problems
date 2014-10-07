
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "plot.h"
#include "oneTruckAllNodesInit.h"



void OneTruckAllNodesInit::stepOne(Vehicle &truck, Bucket &unassigned, Bucket &assigned) {
    if (not unassigned.size()) return;
    Trashnode bestNode;
    UID bestPos;
    if (truck.findNearestNodeTo( unassigned, twc,  bestPos,  bestNode) ) {
        truck.insert(bestNode,bestPos);
        assigned.push_back(bestNode);
        unassigned.erase(bestNode);
        stepOne(truck, unassigned, assigned);
    }
} 







void OneTruckAllNodesInit::process() {

    Bucket unassigned = pickups;
    Bucket assigned;

    assert(not assigned.size());

    std::deque<Vehicle> unusedTrucks = trucks;
    std::deque<Vehicle> usedTrucks = trucks;


    Vehicle truck;

    clearFleet();
    truck=unusedTrucks[0];
    unusedTrucks.erase(unusedTrucks.begin());
    usedTrucks.push_back(truck);


    stepOne(truck,unassigned,assigned);        

truck.plot("OneTruckAllNodes","OneTruckAllNodes",truck.getVid()); // plot displays the route to be plotted
    fleet.push_back(truck);
    assert(pickups ==  assigned);

}
