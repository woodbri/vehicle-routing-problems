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
#include <algorithm>
#include <sstream>
#include <fstream>
#include "twc.h"
#include "twbucket.h"
//#include "plot.h"

#include "truckManyVisitsDump.h"

void TruckManyVisitsDump::insertComming(Bucket &bigTruck, Vehicle &truck,
    UID goingPos )
{
#if 0
  // THE INVARIANT
  // union must be pickups
  assert( pickups == unassigned + problematic + assigned );
  // all intersections must be empty set
  assert( not ( unassigned * problematic ).size()  ) ;
  assert( not ( unassigned * assigned ).size()  ) ;
  assert( not ( problematic * assigned ).size()  ) ;

  //END INVARIANT
  //truck.plot("truckManyVisitsDump-","Insert Comming",tmp++);

  if (not unassigned.size())  {return;};  // something has to be done before return

  assert(bigTruck.size() > 1);

  Trashnode comming;

  comming = bigTruck[bigTruck.size() - 1];

  if ( truck.e_insertSteadyDumpsTight( comming, goingPos + 1 ) ) {
#ifdef DOVRPLOG
    truck.dumpeval();
#endif
    assigned.push_back( comming );
    unassigned.erase( comming );
    bigTruck.erase( comming );
    goingPos++;
  } else {
    //truck.plot( "truckManyVisitsDump-", "Pushing: ", truck.getVid() );
    fleet.push_back( truck );
    truck = getTruck();
    goingPos = 1;
    insertGoing( bigTruck, truck, goingPos );

    return;
  }

  insertGoing( bigTruck, truck, goingPos );
  //the invariant must hold before a return
  assert( pickups == unassigned + problematic + assigned );
  assert( not ( unassigned * problematic ).size()  ) ;
  assert( not ( unassigned * assigned ).size()  ) ;
  assert( not ( problematic * assigned ).size()  ) ;
  return;
#endif
}



void TruckManyVisitsDump::insertGoing( Bucket &bigTruck, Vehicle &truck,
                                       UID goingPos )
{
#if 0
  // THE INVARIANT
  // union must be pickups
  assert( pickups == unassigned + problematic + assigned );
  // all intersections must be empty set
  assert( not ( unassigned * problematic ).size()  ) ;
  assert( not ( unassigned * assigned ).size()  ) ;
  assert( not ( problematic * assigned ).size()  ) ;
  //END INVARIANT

  //truck.plot("truckManyVisitsDump-","Insert Going",tmp++);
  if ( not unassigned.size() ) {
    //asomething has to be done before return
    return;
  };

  assert( bigTruck.size() > 1 );

  Trashnode going;

  going = bigTruck[1];

  // alternate code

#ifdef DOVRPLOG
  truck.dump( "Truck" );

  truck.smalldump();

#endif

  if ( truck.e_insertSteadyDumpsTight( going, goingPos ) ) {
    // the container going is inserted
    assigned.push_back(going);
    unassigned.erase(going);
    bigTruck.erase( 1 );
  } else {
    if (truck.e_insertDumpInPath(going)) { //true when the insertion was performed (no TV)
      goingPos = truck.size() - 1;
      assigned.push_back( going );
      unassigned.erase( going );
      bigTruck.erase( 1 );
    } else {   //adding the Dump and the going node creates a TV
      //we need a new truck
      //truck.plot( "truckManyVisitsDump-", "Many Visits", truck.getVid() );
      fleet.push_back( truck );
      truck = getTruck();
      goingPos = 1;
      insertGoing( bigTruck, truck, goingPos );
      return;
    }
  }

  insertComming( bigTruck, truck, goingPos );
  //the invariant must hold before a return
  assert( pickups == unassigned + problematic + assigned );
  assert( not ( unassigned * problematic ).size()  ) ;
  assert( not ( unassigned * assigned ).size()  ) ;
  assert( not ( problematic * assigned ).size()  ) ;
  return;
#endif
}


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
  assert(pickups == (unassigned + problematic + assigned));
  // all intersections must be empty set
  assert(!(unassigned * problematic).size());
  assert(!(unassigned * assigned).size());
  assert(!(problematic * assigned).size());
  //END INVARIANT

  // preparing a big truck where to store everything
  Vehicle bigTruck = getTruck();
  bigTruck.tau();
  fillOneTruck(bigTruck, unassigned, assigned);

  bigTruck.evaluate();
bigTruck.tau();
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
