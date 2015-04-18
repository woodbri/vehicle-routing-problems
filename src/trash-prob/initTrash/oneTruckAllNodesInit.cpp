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

#ifdef DOPLOT
#include "plot.h"
#endif

#include "oneTruckAllNodesInit.h"

void OneTruckAllNodesInit::stepOne( Vehicle &truck, Bucket &unassigned,
                                    Bucket &assigned )
{
  if ( not unassigned.size() ) return;

  Trashnode bestNode;
  UID bestPos;

  if ( truck.findNearestNodeTo( unassigned,   bestPos,  bestNode ) ) {
    truck.insert( bestNode, bestPos );
    assigned.push_back( bestNode );
    unassigned.erase( bestNode );
    stepOne( truck, unassigned, assigned );
  }
}

void OneTruckAllNodesInit::process()
{

  Bucket unassigned = pickups;
  Bucket assigned;

  assert(not assigned.size());

  std::deque<Vehicle> unusedTrucks = trucks;
  std::deque<Vehicle> usedTrucks = trucks;


  Vehicle truck;

  clearFleet();
  truck = unusedTrucks[0];
  unusedTrucks.erase( unusedTrucks.begin() );
  usedTrucks.push_back( truck );


  stepOne( truck, unassigned, assigned );

  // plot displays the route to be plotted

#ifdef DOPLOT
  truck.plot( "OneTruckAllNodes", "OneTruckAllNodes", truck.getVid() );
#endif

  fleet.push_back( truck );
  assert( pickups ==  assigned );

}
