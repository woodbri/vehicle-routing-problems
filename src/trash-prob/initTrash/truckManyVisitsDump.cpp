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

#include "truckManyVisitsDump.h"

void TruckManyVisitsDump::invariant() {
#if 0
    if ((unassigned.size() + assigned.size()) != 109) {
    DLOG(INFO) << "assigned:" << assigned.size();
    assigned.dumpid();
    DLOG(INFO) << "unassigned" << unassigned.size();
    unassigned.dumpid();
    assert ((unassigned.size() + assigned.size()) == 109);
    }
#endif
    assert(pickups == (unassigned + problematic + assigned));
    assert(!(unassigned * problematic).size());
    assert(!(unassigned * assigned).size());
    assert(!(problematic * assigned).size());
}


void TruckManyVisitsDump::initializeTrip(Vehicle &trip, bool fromStart) {
  invariant();
  trip.evaluate();
#ifdef VRPMINTRACE
  if (!trip.feasable()) {
    trip.dumpeval("NOT FEASABLE in initializeTrip");
    trip.tau("SHOULD BE FEASABLE trip");
    assert(trip.feasable());
  }
#endif
  assert(trip.feasable());
  if (unassigned.size() == 0) return;

  if (fromStart) { 
    UINT bestPos;
    float8 bestTime;
    Trashnode bestNode;
    trip.findFastestNodeTo(true, unassigned, bestPos, bestNode, bestTime); 
    insertBestPairInCleanTrip(trip);
    trip.e_insert(bestNode, 1);
    assigned.push_back(bestNode);
    unassigned.erase(bestNode);
    
    insertNodesOnPath(trip);
  } else {
    insertBestPairInCleanTrip(trip);
  }
  switch (icase) {
    case 1:
      insertBigSubPathAtBegin(trip);
      break;
    default:
      {};
  }
//  insertBigSubPathAtEnd(trip);
//  insertNodesOnPath(trip);
  trip.evaluate();
  trip.tau("initializing step 1");
  remove_CV(trip);
  trip.tau("initializing step 2");
  invariant();
}



bool TruckManyVisitsDump::insertBestPairInCleanTrip(Vehicle &trip) {
  invariant();
  assert(trip.feasable());
  assert(trip.size() == 1);
  if (unassigned.size() == 0) return false;
  UINT bestFrom;
  UINT bestTo;
  Bucket subPath;
  // subpath includes bestFrom and bestTo
  trip.findPairNodesHasMoreNodesOnPath(assigned, unassigned, bestFrom, bestTo, subPath);
  assert(subPath.size() != 0);
#ifdef VRPMMAXTRACE
  subPath.dumpid("BEST SUBPATH FOR INITIALIZING TRIP");
#endif
  trip.insert(subPath, 1);
  assigned = assigned + subPath;
  unassigned = unassigned - subPath;

  trip.getCostOsrm();
#ifdef VRPMMAXTRACE
  trip.tau("after inserting");
#endif
  invariant();
  return true;
}


bool TruckManyVisitsDump::insertBigSubPathAtBegin(Vehicle &trip) {
  invariant();
  if (unassigned.size() == 0) return false;
  UINT bestNode;
  Bucket subPath;
  twc->findBestFromNodeHasMoreNodesOnPath(assigned, unassigned, bestNode, trip[1].nid(), subPath);
  assert(subPath.size() != 0);
#ifdef VRPMAXTRACE
  trip.tau();
  subPath.dumpid("subpath at before");
#endif

  invariant();
  if(subPath.size() == 1) return false;
  trip.insert(subPath, 1);
  assigned = assigned + subPath;
  unassigned = unassigned - subPath;

  trip.getCostOsrm();
#ifdef VRPMAXTRACE
  trip.tau("after inserting");
#endif
  invariant();
  return true;
}

bool TruckManyVisitsDump::insertBigSubPathAtEnd(Vehicle &trip) {
  invariant();
  if (unassigned.size() == 0) return false;
  UINT bestNode;
  Bucket subPath;
  twc->findBestToNodeHasMoreNodesOnPath(assigned, unassigned, trip[trip.size()-1].nid(), bestNode, subPath);
  assert(subPath.size() != 0);
#ifdef VRPMAXTRACE
  trip.tau();
  subPath.dumpid("subpath at after");
#endif
  invariant();
  if(subPath.size() == 1) return false;
  trip.insert(subPath, trip.size());
  assigned = assigned + subPath;
  unassigned = unassigned - subPath;

  trip.getCostOsrm();
#ifdef VRPMAXTRACE
  trip.tau("after inserting" );
#endif
  invariant();
  return true;
}


void TruckManyVisitsDump::fillTrip(Vehicle &trip) {
  invariant();
  UINT oldSize = trip.size();;
  trip.getCostOsrm();
#ifdef VRPMINTRACE
  if (!trip.feasable()) {
     trip.tau("NOT FEASABLE in fillTrip");
     assert(trip.feasable());
  }
#endif
  assert(trip.feasable());
  if (unassigned.size() == 0) return;
  assert(unassigned.size() != 0);
  Trashnode bestNode;
  UINT bestPos;
  Bucket subPath;
#ifdef VRPMINTRACE
  DLOG(INFO) << "trip needs" << trip.getz1() << "nodes";
  trip.tau("before inserting nodes in path");
  // trip.dumpCostValues();
  // if ( trip.getz1() == 9 ) assert(true==false);
#endif
  float8 bestTime;

  Bucket aux;
  // bestNode = unassigned[0];
  trip.findFastestNodeTo(true, unassigned, bestPos, bestNode, bestTime);
  aux.push_back(bestNode);
  invariant();
  trip.findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);
  trip.e_insert(bestNode, bestPos);
  DLOG(INFO) << "bestNode" << bestNode.id();
  DLOG(INFO) << "at pos " << bestPos << " after " << trip[bestPos-1].id();
  assigned.push_back(bestNode);
  unassigned.erase(bestNode);
  insertNodesOnPath(trip);
  invariant();
  trip.tau("after inserting");

 
#ifdef VRPMINTRACE
  trip.tau("after inserting nodes in path");
  DLOG(INFO)  << "oldSize" << oldSize << "  newsize " << trip.size();
  DLOG(INFO) << "assigned " << assigned.size();
  DLOG(INFO) << "unassigned " << unassigned.size();
  trip.evaluate();
#endif

  assert(oldSize < trip.size());
  trip.evaluate();
  if (trip.feasable()) {
    fillTrip(trip); // recursive until its unfeasable
    trip.evaluate();
  }
  trip.evaluate();
  if (trip.feasable()) {
    return; // we ran out of nodes
  }

  trip.evaluate();
#ifdef VRPMINTRACE
  if (trip.feasable()) {
     trip.dumpeval("SHOULD BE INFEASABLE in fillTrip");
     assert(!trip.feasable());
  }
#endif
  assert(!trip.feasable());
  remove_CV(trip);

  if (trip.has_twv()) {
    // need to delete nodes until it doesnt have cv
    trip.dumpeval();
    assert(true==false);
  }
  assert(!trip.has_cv());
  assert(!trip.has_twv());
  assert(trip.feasable());
}

void TruckManyVisitsDump::remove_CV(Vehicle &trip) {
  invariant();
  trip.getCostOsrm();
  if (trip.has_cv()) {
#ifdef VRPMINTRACE
    trip.tau("trip with cv");
    DLOG(INFO) << "assigned"; assigned.dumpid();
    DLOG(INFO) << "unassigned"; unassigned.dumpid();
#endif
    // need to delete nodes until it doesnt have cv
    trip.evaluate();
    while(trip.has_cv()) {
      unassigned.push_back(trip[trip.size()-1]);
      assigned.erase(trip[trip.size()-1]);
      trip.e_remove(trip.size()-1);
      trip.evaluate();
    }
  }
  trip.getCostOsrm();
  assert(!trip.has_cv());
#ifdef VRPMINTRACE
  trip.tau("trip with no cv");
  DLOG(INFO) << "assigned"; assigned.dumpid();
  DLOG(INFO) << "unassigned"; unassigned.dumpid();
#endif
  invariant();
}


void TruckManyVisitsDump::remove_TWV(Vehicle &trip) {
  invariant();
  trip.getCostOsrm();
  if (trip.has_twv()) {
#ifdef VRPMAXTRACE
    DLOG(INFO) << "has twv";
    trip.tau("trip with twv");
#endif
    // need to delete nodes until it doesnt have cv
    trip.evaluate();
    while(trip.has_twv()) {
      unassigned.push_back(trip[trip.size()-1]);
      assigned.erase(trip[trip.size()-1]);
      trip.e_remove(trip.size()-1);
      trip.evaluate();
    }
  }
  trip.getCostOsrm();
  assert(!trip.has_twv());
#ifdef VRPMMAXTRACE
  trip.tau("trip with no twv");
#endif
  invariant();
}

void TruckManyVisitsDump::initializeTruck(Vehicle &truck, std::deque<Vehicle> &trips) {
  invariant();
  assert(unassigned.size() != 0);
  truck.getCostOsrm();
#ifdef VRPMAXTRACE
  truck.dumpCostValues();
  truck.tau("truck:");
#endif
  assert(truck.feasable());
  // prepare the trip
  Vehicle trip = truck;
  if (trip.size() == 1) {
    // this is the first trip
    // nothing to do
    initializeTrip(trip, true);
  } else {
    // startingSite = dumpSite of last trip
    trip.clear();
    trip.set_startingSite(truck.getDumpSite()); 
    initializeTrip(trip, false);
    //trip.dumpeval("initialization of next trip: dump dump end");
  }

  // end preparing the trip
  assert(trip.feasable());

  // we are here because
  if (trip.feasable()) {
    // save trip in truck
    assert(truck.feasable());
    Vehicle test_truck = truck;
    if (test_truck.size() != 1) test_truck.push_back(truck.getDumpSite());
    for (unsigned int i = 1; i < trip.size(); ++i)
      test_truck.push_back(trip[i]);
    test_truck.getCostOsrm();

    if (test_truck.feasable()) {
      truck = test_truck;

      truck.getCostOsrm();
      assert(truck.feasable());

      // find out if another trip is possible 
      trip.getCostOsrm();
      if (trip.getz2() > 0 && truck.getz2() > 0) {
        // another trip is possible
        // add a dump to the truck
        assert(truck.feasable());

        // saving trip start containers dump dump
        trip.getCostOsrm();
        trip.tau("trip to save before changing ending to be dump");
        assert(trip.feasable());
        trip.set_endingSite(truck.getDumpSite());
        trip.getCostOsrm();
        trip.tau("trip to save after changing ending to be dump");
        trips.push_back(trip);
        assert(trip.feasable());
        // finished saving trip

        // continue filling the truck
        if (unassigned.size() != 0) initializeTruck(truck, trips);
      } else {
        // adding a new trip is not possible
        trip.tau("trip to save dosnt change ending because we are not inserting new trip");
        trips.push_back(trip);
      }
    } else {
      // adding the trip makes it infeasable
      // clear the trip
#ifdef VRPMAXTRACE
      truck.tau("truck of trip made infeasable truck");
      trip.tau("trip made infeasable truck");
#endif
      deleteTrip(trip);
    }
  }
  truck.getCostOsrm();
  assert(truck.feasable());
  invariant();
}

// trip is no longer usable
void TruckManyVisitsDump::deleteTrip(Vehicle &trip) {
  // remove last trip to give time to trips to work
  invariant();
#ifdef VRPMINTRACE
    DLOG(INFO) << "assigned"; assigned.dumpid();
    DLOG(INFO) << "unassigned"; unassigned.dumpid();
  trip.tau("trip to be removed");
#endif
  Bucket nodes;
  for (UINT i = 1; i < trip.size(); ++i) {
    nodes.push_back(trip[i]);
  }
  unassigned = unassigned + nodes;
  assigned = assigned - nodes;
#ifdef VRPMINTRACE
    DLOG(INFO) << "assigned";
    assigned.dumpid();
    DLOG(INFO) << "unassigned";
    unassigned.dumpid();
#endif
  invariant();
}


void TruckManyVisitsDump::fillTruck(Vehicle &truck, std::deque<Vehicle> &trips) {
  invariant();
  assert(unassigned.size() != 0);
  initializeTruck(truck, trips);
#ifdef VRPMAXTRACE
  truck.tau("final truck initialization");
  truck.dumpCostValues();
  for (UINT i = 0; i < trips.size(); ++i) {
    trips[i].getCostOsrm();
    trips[i].dumpCostValues();
  }
  for (UINT i = 0; i < trips.size(); ++i) {
    trips[i].tau();
  }
#endif

  deleteTrip(trips[trips.size()-1]);
  trips.pop_back();
#if 1
  //iterate thru the trips to fill them
  for (UINT i = 0; i < trips.size(); ++i) {
#ifdef VRPMAXTRACE
    DLOG(INFO) << "insertNodesOnPath trip: " << i;
    trips[i].tau(" before");
#endif
    insertNodesOnPath(trips[i]);
    remove_CV(trips[i]);
    assert(!trips[i].has_cv());
    trips[i].tau(" after");
  }

  if (unassigned.size() > 0) {
    for (UINT i = 0; i < trips.size(); ++i) {
#ifdef VRPMINTRACE
      DLOG(INFO) << "Fill trip: " << i;
      trips[i].tau(" before");
#endif
      if (trips[i].getz1() > 0 && trips[i].feasable() && unassigned.size()) {
        trips[i].getCostOsrm();
        fillTrip(trips[i]);
      }
      trips[i].tau(" after");
    }
  }
#endif

  buildTruck(truck, trips);

  invariant();
}

// based on the trips we build the truck
// if 
void TruckManyVisitsDump::buildTruck(Vehicle &truck, std::deque<Vehicle> &trips) {
  truck.e_clean();
  for (UINT i = 0; i < trips.size(); ++i) {
    trips[i].getCostOsrm();
    for (UINT j = 1; j < trips[i].size(); ++j) {
      truck.push_back(trips[i][j]);
    }
    if (i != trips.size() - 1) truck.push_back(truck.getDumpSite());   
  }
  truck.getCostOsrm();
  truck.tau("before adjust dumps");
  truck.e_adjustDumpsToNoCV(1);
  truck.getCostOsrm();
  truck.tau("before adjust adjust time window");
  remove_TWV(truck);
  truck.getCostOsrm();
  truck.tau("after adjust adjust time window");
  // once the truck is built need to reconstruct the trips
  // and adjust them to limit the time windows
  // so that on trip is after the other
  assert(truck.feasable());
  invariant();
}

void TruckManyVisitsDump::fillFleet() {
  invariant();
  int i = 0;
  while (unassigned.size() > 0) {
    DLOG(INFO) << "-+-+-+-+-+-+-+-+-+-+-+-+-+-+- TRUCK " << i << " +-+-+-+-+-+-+-+-+-+-+-+-+-+-";
    Vehicle truck;
    std::deque<Vehicle> trips;
    truck = getTruck();
    trips.clear();
    fillTruck(truck, trips);
// assert(true==false);
#if 1
    fleet.push_back(truck);
#else
    for (UINT i = 0; i < trips.size(); ++i) {
      fleet.push_back(trips[i]);
    }
#endif
   ++i; 
  }
  invariant();
  assert(unassigned.size() == 0);
}

void TruckManyVisitsDump::insertNodesOnPath(Vehicle &trip) {
  Bucket streetNodes, tmp;
  invariant();

  streetNodes.clear();
  twc->getNodesOnPath(trip.Path(), trip.getDumpSite(), unassigned, streetNodes);
  tmp = streetNodes;
  assert((tmp * assigned).size() == 0);
  Bucket aux;
  Trashnode bestNode;
  float8 bestTime;
  UINT bestPos;
    // insert the containers that are in the path
  int lastBestPos = 0;
  while (streetNodes.size() > 0) {
        invariant();
        aux.clear();
        aux.push_back(streetNodes[0]);
        streetNodes.erase(streetNodes[0]);
        trip.findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);
#ifdef VRPMINTRACE
        DLOG(INFO) << "trying inserting node in path: " << bestNode.id() << " at position " << bestPos;
#endif
        if (bestPos <= lastBestPos) bestPos++;
        trip.e_insert(bestNode, bestPos);
        lastBestPos = bestPos;
        trip.tau("inserted on path");
        assigned.push_back(bestNode);
        unassigned.erase(bestNode);
        invariant();
  }
  invariant();
}



/*
  \Return true: New truck form the trucks bucket
  \Return fasle: a clean copy of the last truck when bucket is empty
*/
Vehicle  TruckManyVisitsDump::getTruck() {
  assert(unusedTrucks.size() > 0);
  Vehicle  truck = unusedTrucks[0];
  truck.getCostOsrm();
  usedTrucks.push_back(truck);
  if (unusedTrucks.size() > 1) {
    unusedTrucks.erase(unusedTrucks.begin());
  }
  assert(unusedTrucks.size() > 0);
  return truck;
}


#if 0
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
    // find a node that container that inserts the most containers
    // we want to insert many many containers
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
#endif

//    PROCESS
//
//    This implements a feasable solution

void TruckManyVisitsDump::process(int pcase)
{
  unusedTrucks = trucks;
  usedTrucks.clear();
  assigned.clear();
  unassigned = pickups;
  fleet.clear();
  icase = pcase;
#ifdef VRPMINTRACE
  // THE INVARIANT
  // union must be pickups
  assert(pickups == (unassigned + problematic + assigned));
  assert(!(unassigned * problematic).size());
  assert(!(unassigned * assigned).size());
  assert(!(problematic * assigned).size());
  //END INVARIANT
#endif

  DLOG(INFO) << "Starting initial Solution Proccess\n";
  bool oldState = osrmi->getUse();
  osrmi->useOsrm(true);
  fillFleet();

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
