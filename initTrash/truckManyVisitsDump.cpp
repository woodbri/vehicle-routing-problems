
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "plot.h"
#include "truckManyVisitsDump.h"
//#include "oneTruckAllNodesInit.h"

void TruckManyVisitsDump::insertComming(Bucket &bigTruck,Vehicle &truck, UID goingPos) {
// THE INVARIANT
// union must be pickups
    assert(pickups == unassigned + problematic + assigned);
// all intersections must be empty set
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
//END INVARIANT

    if (not unassigned.size()) { return; }; //something has to be done before return
    Trashnode comming;
    comming=bigTruck[bigTruck.size()-1];
    if ( truck.e_insertSteadyDumpsTight(comming,goingPos+1) ) {
            assigned.push_back(comming);
            unassigned.erase(comming);
            bigTruck.erase(comming);
            goingPos++; //next recurtion value
        } else {
        // we need a new truck
/*
                fleet.push_back(truck);
                turck=getTruck();
                goingPos=1;
                insertGoing(bigTruck,truck,goingPos);
*/              
                return;
    }
    insertGoing(bigTruck,truck,goingPos);
    //the invariant must hold before a return
    assert(pickups == unassigned + problematic + assigned);
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
    return;
}



//   

void TruckManyVisitsDump::insertGoing(Bucket &bigTruck,Vehicle &truck, UID goingPos) {
// THE INVARIANT
// union must be pickups
    assert(pickups == unassigned + problematic + assigned);
// all intersections must be empty set
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
//END INVARIANT
    if (not unassigned.size()) { return; }; //asomething has to be done before return
    Trashnode going;
    going=bigTruck[1];
// alternate code

         
    if ( truck.e_insertSteadyDumpsTight(going,goingPos+1) ) {
            truck.insert(going,goingPos);
            assigned.push_back(going);
            unassigned.erase(going);
    } else {

/*
            if ( truck.InsertDumpInPath( going ) ) { //true when the insertion was performed (no TV)
        	goingPos=truck.size()-1;  
                assigned.push_back(going);
                unassigned.erase(going);
            }  else {  //adding the Dump and the going node creates a TV
	      //we need a new truck 
		fleet.push_back(truck);
		turck=getTruck();
                goingPos=1;
		insertGoing(bigTruck,truck,goingPos);
		return;
            }
	}
*/
    }
    insertComming(bigTruck,truck,goingPos);
    //the invariant must hold before a return
    assert(pickups == unassigned + problematic + assigned);
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
    return;
} 




Vehicle  TruckManyVisitsDump::getTruck() {
        Vehicle truck=unusedTrucks[0];
        unusedTrucks.erase(unusedTrucks.begin());
        usedTrucks.push_back(truck);
        return truck;
}


//    PROCESS
//
//    This implements a feasable solution

void TruckManyVisitsDump::process() {
// THE INVARIANT
// union must be pickups
    assert(pickups == unassigned + problematic + assigned);
// all intersections must be empty set
    assert( not (unassigned * problematic).size()  ) ;
    assert( not (unassigned * assigned).size()  ) ;
    assert( not (problematic * assigned).size()  ) ;
//END INVARIANT

    //reteriving the result of oneTruckAllNodesInit
    Bucket bigTruck;
    bigTruck=fleet[0].Path();
    fleet.clear();
    int goingPos=1;
    Vehicle truck;
    truck=getTruck();

    insertGoing(bigTruck,truck,goingPos);        
//    fleet.push_back(truck); //need to save the last truck

truck.plot("Feasable-","",truck.getVid());
return;
}
