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

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "timer.h"
#include "plot.h"
#endif

#include "feasableSol.h"




//   need new truck when bestNode generates TV regardless of cargo
//   what to check firts Cargo or Time????
//   how to handla that once the Dump is inserted, not to look for best Position on
//       the first part of ther Route????

void FeasableSol::stepOne( Vehicle &truck )
{
  // THE INVARIANT
  // union must be pickups
  assert( pickups == unassigned + problematic + assigned );
  // all intersections must be empty set
  assert( not ( unassigned * problematic ).size()  ) ;
  assert( not ( unassigned * assigned ).size()  ) ;
  assert( not ( problematic * assigned ).size()  ) ;
  assert ( truck.feasable() ) ;
  //END INVARIANT

  if ( not unassigned.size() ) return;

  Trashnode bestNode;
  UID bestPos;

  if ( truck.findNearestNodeTo( unassigned,   bestPos,  bestNode ) ) {
    if (  not  truck.e_insertIntoFeasableTruck( bestNode, bestPos ) ) {
      fleet.push_back( truck );

      if ( unusedTrucks.size() == 0 ) return;

      truck = unusedTrucks[0];
      unusedTrucks.erase( unusedTrucks.begin() );
      usedTrucks.push_back( truck );

      assert ( truck.feasable() ) ;
      stepOne( truck );
    } else {
      truck.e_insert(bestNode,bestPos);
      assigned.push_back( bestNode );
      unassigned.erase( bestNode );

      assert ( truck.feasable() ) ;
      stepOne( truck );
    }
  } else {
#ifdef DOVRPLOG
    DLOG( WARNING ) << " FeasableSol::stepOne: No nearest node was found";
#endif
    assert( std::string("FeasableSol::stepOne") ==
            std::string("no nearest node was found"));
  }

}




Vehicle  FeasableSol::getTruck()
{
  assert( unusedTrucks[0].size() );

  Vehicle truck = unusedTrucks[0];
  unusedTrucks.erase( unusedTrucks.begin() );
  usedTrucks.push_back( truck );
  return truck;
}


//    PROCESS
//
//    This implements a feasable solution

void FeasableSol::process()
{
  // THE INVARIANT
  // union must be pickups
  assert( pickups == unassigned + problematic + assigned );
  // all intersections must be empty set
  assert( not ( unassigned * problematic ).size()  ) ;
  assert( not ( unassigned * assigned ).size()  ) ;
  assert( not ( problematic * assigned ).size()  ) ;
  //END INVARIANT
#ifdef DOSTATS
  Timer start;
#endif

  Vehicle truck;
  truck = getTruck();
  stepOne( truck );
  fleet.push_back( truck ); //need to save the last truck

#ifdef DOSTATS
  DLOG( INFO ) << "FEASABLESOL: Total time: " << start.duration();
#endif
  return;
}
