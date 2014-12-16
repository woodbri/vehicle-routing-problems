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

#include "logger.h"
#include "plot.h"
#include "feasableSolLoop.h"




//   need new truck when bestNode generates TV regardless of cargo
//   what to check firts Cargo or Time????
//   how to handla that once the Dump is inserted,
//      not to look for best Position on
//      the first part of ther Route????

void FeasableSolLoop::stepOneLoop( Vehicle &truck ) {
    int iteration = 0;

    while ( unassigned.size() ) {
        // THE INVARIANT
        // union must be pickups
        assert( pickups == unassigned + problematic + assigned );
        // all intersections must be empty set
        assert( not ( unassigned * problematic ).size()  ) ;
        assert( not ( unassigned * assigned ).size()  ) ;
        assert( not ( problematic * assigned ).size()  ) ;
        assert ( truck.feasable() ) ;
        //END INVARIANT


        Trashnode bestNode;
        UID bestPos;

        if ( truck.findNearestNodeTo( unassigned,  bestPos, bestNode ) ) {
            if ( not  truck.e_insertIntoFeasableTruck( bestNode, bestPos ) ) {
                fleet.push_back( truck );
                truck.plot( "Feasable-", "", truck.getVid() );

                if ( unusedTrucks.size() ) {
                    truck = unusedTrucks[0];
                    unusedTrucks.erase( unusedTrucks.begin() );
                    usedTrucks.push_back( truck );
                }
                else {
                    DLOG( INFO ) << "No more trucks available. unassigned containers: " <<
                                 unassigned.size();
                    return;
                }

            }
            else {
                assigned.push_back( bestNode );
                unassigned.erase( bestNode );
            }
        }
        else {
            DLOG( WARNING ) << "no nearest node was found";
            assert( "FeasableSolLoop::stepOneLoop" == "no nearest node was found" );
        }

        ++iteration;
    }
}




Vehicle  FeasableSolLoop::getTruck() {
    Vehicle truck = unusedTrucks[0];
    unusedTrucks.erase( unusedTrucks.begin() );
    usedTrucks.push_back( truck );
    return truck;
}


//    PROCESS
//
//    This implements a feasable solution

void FeasableSolLoop::process() {
    // THE INVARIANT
    // union must be pickups
    assert( pickups == unassigned + problematic + assigned );
    // all intersections must be empty set
    assert( not ( unassigned * problematic ).size()  ) ;
    assert( not ( unassigned * assigned ).size()  ) ;
    assert( not ( problematic * assigned ).size()  ) ;
    //END INVARIANT

    Vehicle truck;

    truck = getTruck();

    stepOneLoop( truck );
    fleet.push_back( truck ); //need to save the last truck

    truck.plot( "Feasable-", "", truck.getVid() );
    return;
}
