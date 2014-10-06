
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "plot.h"
#include "feasableSol.h"




//   need new truck when bestNode generates TV regardless of cargo
//   what to check firts Cargo or Time????
//   how to handla that once the Dump is inserted, not to look for best Position on
//       the first part of ther Route????

void FeasableSol::stepOne(Vehicle &truck) {
// THE INVARIANT
// union must be pickups
    assert(pickups == unassigned + problematic + assigned);
// all intersections must be empty set
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
//END INVARIANT

    if (not unassigned.size()) return;
          
    Trashnode bestNode;
    UID bestPos;
    if (truck.findNearestNodeTo( unassigned, twc,  bestPos,  bestNode) ) {
        if(  truck.deltaCargoGeneratesCV(bestNode) ) {
            if (truck.deltaTimeGeneratesTV(bestNode,bestPos)) {}; //TBD
                fleet.push_back(truck);
truck.plot("Feasable-","",truck.getVid());

                truck=unusedTrucks[0];
                unusedTrucks.erase(unusedTrucks.begin());
                usedTrucks.push_back(truck);

                stepOne(truck);
        } else {
            truck.insert(bestNode,bestPos);
            assigned.push_back(bestNode);
            unassigned.erase(bestNode);
            stepOne(truck);
        } 
    }
} 




Vehicle  FeasableSol::getTruck() {
        Vehicle truck=unusedTrucks[0];
        unusedTrucks.erase(unusedTrucks.begin());
        usedTrucks.push_back(truck);
        return truck;
}


//    PROCESS
//
//    This implements a feasable solution

void FeasableSol::process() {
// THE INVARIANT
// union must be pickups
    assert(pickups == unassigned + problematic + assigned);
// all intersections must be empty set
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
//END INVARIANT

    Vehicle truck;

    truck=getTruck();

    stepOne(truck);        
    fleet.push_back(truck); //need to save the last truck

truck.plot("Feasable-","",truck.getVid());
return;
}
