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

//#include <limits>
//#include <stdexcept>
//#include <string>
//#include <iostream>
//#include <iomanip>
//#include <algorithm>
//#include <sstream>
//#include <fstream>
//#include "twc.h"
//#include "twbucket.h"
#include "truckManyVisitsDump.h"


void TruckManyVisitsDump::fillOneTruck(
         Vehicle &truckToBeFilled) {  // truck to be filled
#ifdef VRPMINTRACE
  assert(pickups == (unassigned + problematic + assigned));
#endif
  
  // nothing left to be assigned
  if (unassigned.size() == 0) return;

  Trashnode bestNode;
  UID bestPos;
  double bestTime;
  Bucket streetNodes, tmp;
  Bucket unassignedStreetNodes;
  Bucket aux;
  uint64_t  street_id;
  bool first = true;
  Vehicle trip = truckToBeFilled; 
  bool cv_flag = false;

DLOG(INFO) << " STARTING\n ";
  while (unassigned.size() != 0) {
#ifdef VRPMINTRACE
  assert(pickups == (unassigned + problematic + assigned));
#endif

    THROW_ON_SIGINT

    // find a costly node
    if (trip.findFastestNodeTo(true, unassigned, bestPos, bestNode, bestTime)) {
      aux.clear();
      aux.push_back(bestNode);
      // of the costly node find the cheaper position
      trip.findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);

      trip.e_insert(bestNode, bestPos);

      if (trip.has_cv()) {
        trip.e_remove(bestPos);
        insertTrip(trip, truckToBeFilled);
        continue;
      } else {
#ifdef VRPMINTRACE
        DLOG(INFO) << "inserting a node: " << bestNode.id();
        trip.tau();
#endif
        assigned.push_back(bestNode);
        unassigned.erase(bestNode);
        cv_flag = false;
      }

      // get containers that are in the path
      streetNodes.clear();
      twc->getNodesOnPath(trip.Path(), trip.getDumpSite(), unassigned, streetNodes);

      // insert the containers that are in the path
      int lastBestPos = 0;
      while (streetNodes.size() > 0) {
#ifdef VRPMINTRACE
        assert(pickups == (unassigned + problematic + assigned));
#endif

        THROW_ON_SIGINT

        aux.clear();
        aux.push_back(streetNodes[0]);
        streetNodes.erase(streetNodes[0]);
        
        trip.findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);
        // insert only nodes that dont change the structure of the path ???  some nodes change
        float oldTime = trip.getDumpSite().totTravelTime();
#ifdef VRPMINTRACE
        DLOG(INFO) << "trying inserting node in path: " << bestNode.id() << " at position " << bestPos;
#endif
        if (bestPos <= lastBestPos) bestPos++;
        trip.e_insert(bestNode, bestPos);
        float newTime = trip.getDumpSite().totTravelTime();
       
        if (trip.has_cv()) {
#ifdef VRPMINTRACE
        DLOG(INFO) << " ---> failed generated a cv: " << bestNode.id();
#endif
          trip.e_remove(bestPos);
          insertTrip(trip, truckToBeFilled);
          streetNodes.clear(); 
#ifdef VRPMINTRACE
          assert(pickups == (unassigned + problematic + assigned));
#endif
          break;
        }  

#ifdef VRPMINTRACE
        DLOG(INFO) << "newtime = " << newTime;
        DLOG(INFO) << "oldtime = " << oldTime;
        DLOG(INFO) << "differe = " << newTime-oldTime;
#endif
        if ((newTime - oldTime) > 0.3 ) {
#ifdef VRPMINTRACE
        DLOG(INFO) << " ---> failed The structure of the path changed: " << bestNode.id();
#endif
          trip.e_remove(bestPos);
        } else {
#ifdef VRPMINTRACE
        DLOG(INFO) << " ---> success: " << bestNode.id();
        lastBestPos = bestPos;
        trip.tau();
#endif
          assigned.push_back(bestNode);
          unassigned.erase(bestNode);
        }

        if (trip.getz1() <= trip.size()-1) {
           insertTrip(trip, truckToBeFilled);
           streetNodes.clear(); 
#ifdef VRPMINTRACE
           assert(pickups == (unassigned + problematic + assigned));
#endif
           break;
        }
      }  // while inserting with cost 0
#ifdef VRPMINTRACE
      assert(pickups == (unassigned + problematic + assigned));
      assert(streetNodes.size() == 0);
#endif

      first = false;
    } else break;
  }

  // we got out either, because of CV or because we dont have more containers
  insertTrip(trip, truckToBeFilled);
  fleet.push_back(truckToBeFilled);
  truckToBeFilled.dumpeval(); 
  assert(unassigned.size()==0);
  assert(assigned == pickups);
  assert(countPickups() == pickups.size());
}


bool TruckManyVisitsDump::insertTrip(
     Vehicle &trip,
     Vehicle &truckToBeFilled) {
  Vehicle truckDuplicate = truckToBeFilled;
  bool gotNewTruck = false;


#ifdef VRPMINTRACE
DLOG(INFO) << " filling trip ";
#endif
  // add a dump if its not the first trip
  if (truckToBeFilled.size() > 1)
    truckToBeFilled.push_back(truckToBeFilled.getDumpSite());

  // insert the trip
  for (unsigned int i = 1; i < trip.size(); ++i)
    truckToBeFilled.push_back(trip[i]);

  if (!truckToBeFilled.feasable()) { 
    DLOG(INFO) << " NOT feasable";
    // TODO fix the last trip
    //truckToBeFilled.dumpeval();
    // if this happens then revise the CV before calling 
    // the function has to be revised
    //assert(!truckToBeFilled.has_cv());
    //the violation then should be twv
    //assert(truckToBeFilled.has_twv());
#ifdef VRPMINTRACE
    if (truckToBeFilled.has_twv()) { 
      DLOG(INFO) << " HAS TWV";
    }
#endif
    // The violation is twv
    // insert the non vioalating truck into the fleet 
    truckDuplicate.evaluate();
    fleet.push_back(truckDuplicate);

    // move from assigned to unassigned all nodes of trip
    // to unassigned all nodes of trip
    for(unsigned int i = 1; i < trip.size(); ++i) {
      assigned.erase(trip[i]);
      unassigned.push_back(trip[i]);
    }
    //  get a new truck
    truckToBeFilled  = getTruck();
    // the trip is the first trip
    trip = truckToBeFilled;
    assert(trip.size()==1);
    assert(truckToBeFilled.size()==1);
  } else {
    // Insert the truck with the new trip
    truckToBeFilled.evaluate();
    //fleet.push_back(truckToBeFilled);

    // Dumpsite of the last trip is starting site of next trip
    Trashnode startS = truckToBeFilled.getDumpSite();
    startS.set_x(trip.getDepot().x());
    startS.set_y(trip.getDepot().y());
    startS.set_type(Twnode::kStart);
    startS.set_demand(0);
    trip.clear(); // this clear removes the starting site of the path
    trip.push_back(startS);
  }
  return true;
}

/*
  \Return true: New truck form the trucks bucket
  \Return fasle: a clean copy of the last truck when bucket is empty
*/
Vehicle  TruckManyVisitsDump::getTruck() {
  assert(unusedTrucks.size() > 0);
  Vehicle  truck = unusedTrucks[0];
  truck.getCost();
  usedTrucks.push_back(truck);
  if (unusedTrucks.size() > 1) {
    unusedTrucks.erase(unusedTrucks.begin());
  }
  assert(unusedTrucks.size() > 0);
  return truck;
}


//    PROCESS
//
//    This implements a feasable solution

void TruckManyVisitsDump::process()
{
#ifdef VRPMINTRACE
  // THE INVARIANT
  // union must be pickups
  assert(pickups == (unassigned + problematic + assigned));
  // all intersections must be empty set
  assert(!(unassigned * problematic).size());
  assert(!(unassigned * assigned).size());
  assert(!(problematic * assigned).size());
  //END INVARIANT
#endif

  DLOG(INFO) << "Starting initial Solution Proccess\n";
  bool oldState = osrmi->getUse();
  osrmi->useOsrm(true);
  Vehicle truck;
  truck = getTruck();
  //truck.getCost();
  fillOneTruck(truck);
  osrmi->useOsrm(oldState);
 
  dumpCostValues();
#ifdef VRPMINTRACE
  DLOG(INFO) << "number of trucks in solition" << fleet.size();
#endif

#ifdef VRPMINTRACE
  // THE INVARIANT
  // union must be pickups
  assert(pickups == (unassigned + problematic + assigned));
  // all intersections must be empty set
  assert(!(unassigned * problematic).size());
  assert(!(unassigned * assigned).size());
  assert(!(problematic * assigned).size());
  //END INVARIANT
#endif
}
