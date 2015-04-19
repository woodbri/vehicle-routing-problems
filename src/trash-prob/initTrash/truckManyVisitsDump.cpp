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

#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <fstream>
#include "twc.h"
#include "twbucket.h"
//#include "plot.h"

#include "truckManyVisitsDump.h"


// evalSwap position i with position j on the same truck
// j > i
//  ....  i-1 i i+1 ..... j-1 j j+1
// esto va en twPath
double TruckManyVisitsDump::e_evalIntraSw(Vehicle &truck, POS i, POS j){
  assert(i > 0);
  assert(j > i);
  assert(j < truck.size()-1);

  if (truck[i].isDump() || truck[j].isDump())
      return VRP_MAX();
  double deltaTravelTime;
  if (j > i + 1) {
    double originalTravelTimeI = twc->TravelTime(truck[i-1], truck[i], truck[i+1]);
    double originalTravelTimeJ = twc->TravelTime(truck[j-1], truck[j], truck[j+1]);
    double newTravelTimeI = twc->TravelTime(truck[i-1], truck[j], truck[i+1]);
    double newTravelTimeJ = twc->TravelTime(truck[j-1], truck[i], truck[j+1]);
    deltaTravelTime = (newTravelTimeI + newTravelTimeJ)
                   - (newTravelTimeI - newTravelTimeI);

  
#ifdef VRPMAXTRACE
    DLOG(INFO) << std::fixed << std::setprecision(4);
    DLOG(INFO) << "travelTime (" << i-1 <<" , " << i << " , " << i + 1 << ")= " 
               << originalTravelTimeI;
    DLOG(INFO) << "travelTime (" << j-1 <<" , " << j << " , " << j + 1 << ")= " 
               << originalTravelTimeJ;
    DLOG(INFO) << "travelTime (" << i-1 <<" , " << j << " , " << i + 1 << ")= " 
               << newTravelTimeI;
    DLOG(INFO) << "travelTime (" << j-1 <<" , " << i << " , " << j + 1 << ")= " 
               << newTravelTimeJ;
    DLOG(INFO)  << "Delta TravelTime: " << deltaTravelTime;
#endif
  } else { // i-1 (i == j-1) (i + 1 == j) j+1 son contiguos
    double originalTravelTime = twc->TravelTime(truck[i-1], truck[i], truck[j], truck[j+1]);
    double newTravelTime = twc->TravelTime(truck[i-1], truck[j], truck[i], truck[j+1]);
    deltaTravelTime = newTravelTime - originalTravelTime;
#ifdef VRPMAXTRACE
    DLOG(INFO) << std::fixed << std::setprecision(4);
    DLOG(INFO) << "travelTime (" << i-1 <<" , " << i << " , " << j << " , " << j + 1 << ")= " 
               << originalTravelTime;
    DLOG(INFO) << "travelTime (" << i-1 <<" , " << j << " , " << i << " , " << j + 1 << ")= " 
               << newTravelTime;
#endif

  }
  return deltaTravelTime;
};

void TruckManyVisitsDump::IntraSwMoves(Vehicle &truck) {
  double deltaTravelTime;
  for (POS i = 1; i < truck.size(); ++i) {
    for (POS j = i + 1; j < truck.size()-1; ++j) {
      deltaTravelTime = e_evalIntraSw(truck,i,j);
      if (deltaTravelTime == VRP_MAX()) continue;
      //if (deltaTravelTime < 0)
        DLOG(INFO) << "delete: " << i <<" insert after: " << j << " = "
                   << "Delta TravelTime: " << deltaTravelTime;
    }
  }
}

void TruckManyVisitsDump::InsMoves(Vehicle &truck) {
  double deltaTravelTime;
  for (POS i = 1; i < truck.size(); ++i) {
    for (POS j = i + 1; j < truck.size()-1; ++j) {
      deltaTravelTime = e_evalIns(truck,i,j);
      if (deltaTravelTime == VRP_MAX()) continue;
      //if (deltaTravelTime < 0)
        DLOG(INFO) << "swap: (" << i <<" , " << j << ")= "
                   << "Delta TravelTime: " << deltaTravelTime;
    }
  }
}

// delteting node at position i, inserting it after node in position j
// 0 ..... i-1  i  i+1 ...       j-1 j  j+1
// 0 ......i-1 i+1 ......    j-1  j  i  j+1    

double TruckManyVisitsDump::e_evalIns(Vehicle &truck, POS i, POS j){
  assert(i > 0);
  assert(j > i);
  assert(j < truck.size()-1);

  if (truck[i].isDump()) // not deleting a dump
      return VRP_MAX();
  double deltaTravelTime;
  // if (j > i + 1) {
    double originalTravelTimeI = twc->TravelTime(truck[i-1], truck[i], truck[i+1]);
    double originalTravelTimeJ = twc->TravelTime(truck[j-1], truck[j], truck[j+1]);
    double newTravelTimeI = twc->TravelTime(truck[i-1], truck[i+1]);
    double newTravelTimeJ = twc->TravelTime(truck[j-1], truck[j], truck[i], truck[j+1]);
    deltaTravelTime = (newTravelTimeI + newTravelTimeJ)
                   - (newTravelTimeI - newTravelTimeI);


#ifdef VRPMAXTRACE
    DLOG(INFO) << std::fixed << std::setprecision(4);
    DLOG(INFO) << "travelTime (" << i-1 <<", " << i << ", " << i + 1 << ")= "
               << originalTravelTimeI;
    DLOG(INFO) << "travelTime (" << j-1 <<", " << j << ", " << j + 1 << ")= "
               << originalTravelTimeJ;
    DLOG(INFO) << "travelTime (" << i-1 <<", " << i + 1 << ")= "
               << newTravelTimeI;
    DLOG(INFO) << "travelTime (" << j-1 <<", " << j << ", "<< i << ", " << j + 1 << ")= "
               << newTravelTimeJ;
    DLOG(INFO)  << "Delta TravelTime: " << deltaTravelTime;
#endif
#if 0
  } else { // i-1 (i == j-1) (i + 1 == j) j+1 son contiguos
    double originalTravelTime = twc->TravelTime(truck[i-1], truck[i], truck[j], truck[j+1]);
    double newTravelTime = twc->TravelTime(truck[i-1], truck[j], truck[i], truck[j+1]);
    deltaTravelTime = newTravelTime - originalTravelTime;
#ifdef VRPMAXTRACE
    DLOG(INFO) << std::fixed << std::setprecision(4);
    DLOG(INFO) << "travelTime (" << i-1 <<" , " << i << " , " << j << " , " << j + 1 << ")= "
               << originalTravelTime;
    DLOG(INFO) << "travelTime (" << i-1 <<" , " << j << " , " << i << " , " << j + 1 << ")= "
               << newTravelTime;
#endif

  }
#endif
  return deltaTravelTime;
};





void TruckManyVisitsDump::fillOneTruck(
         Vehicle &truck,       // truck to be filled
         Bucket &unassigned,   // unassigned containers
         Bucket &assigned ) {  // assigned containers
  
  // nothing left to be assigned
  if (unassigned.size() == 0) return;

  Trashnode bestNode;
  UID bestPos;
  double bestDist;

  while (unassigned.size() != 0) {

    if (twc->findBestTravelTime(truck[truck.size()-1], unassigned, bestNode)) {
      truck.push_back(bestNode);
      assigned.push_back(bestNode);
      unassigned.erase(bestNode);
    } else break;
  }
  truck.dumpeval();
  truck.e_makeFeasable(0);
  truck.dumpeval();
  truck.getCost();
  truck.dumpCostValues();
}



Vehicle  TruckManyVisitsDump::getTruck()
{
  Vehicle truck = unusedTrucks[0];
  unusedTrucks.erase( unusedTrucks.begin() );
  usedTrucks.push_back( truck );
  return truck;
}


//    PROCESS
//
//    This implements a feasable solution

void TruckManyVisitsDump::process()
{
  // THE INVARIANT
  // union must be pickups
#ifdef VRPMAXTRACE
  assert(pickups == (unassigned + problematic + assigned));
  // all intersections must be empty set
  assert(!(unassigned * problematic).size());
  assert(!(unassigned * assigned).size());
  assert(!(problematic * assigned).size());
  //END INVARIANT
#endif

  twc->dump();
  pickups.dumpid("pickups");
  unassigned.dumpid("unassigned");
  // preparing a big truck where to store everything
  Vehicle bigTruck = getTruck();
  bigTruck.tau();
  fillOneTruck(bigTruck, unassigned, assigned);

  bigTruck.evaluate();
bigTruck.tau();
  IntraSwMoves(bigTruck);
  InsMoves(bigTruck);
assert(true==false);
#if 0
  assert(fleet.size());
#ifdef DOVRPLOG
  fleet[0].dump( "fleet[0]" );
#endif
  bigTruck = fleet[0].Path();
  fleet.clear();
  bigTruck.dump( "bigTruck" );
  int goingPos = 1;
  Vehicle truck;
  truck = getTruck();
#ifdef DOVRPLOG
  truck.dump( "Truck" );
#endif
  insertGoing( bigTruck, truck, goingPos );
  fleet.push_back( truck ); //need to save the last truck ??


  //truck.plot( "truckManyVisitsDump-", "Many Visits", truck.getVid() );
  return;
#endif
}
