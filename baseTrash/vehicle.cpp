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


#include <iostream>
#include <sstream>
#include <deque>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "stats.h"
#include "timer.h"
#endif

#include "trashconfig.h"
#include "twpath.h"

#include "move.h"
#include "vehicle.h"
#include "basevehicle.h"


/** \todo comments (see comments of twbucket is the same thing)
   the ifs are for: if just by traveling we get a twv why bother inspecting with more detail
   to be used with intraSw

   prev curr next
   ttpc + serv(c) + ttcn
   tw checks
   infinity when twc
   no cv checks
*/
double Vehicle::timePCN( POS from, POS middle, POS to ) const  {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::timePCN positions" );
    #endif

    if ( to == size() ) {
        if ( ( middle == ( from + 1 ) )
             and ( to == ( middle + 1 ) ) ) return dumpSite.getArrivalTime() -
                         path[from].getDepartureTime();

        if ( dumpSite.lateArrival(  path[from].getDepartureTime() + twc->TravelTime(
                                        path[from], path[middle], dumpSite ) ) ) return _MAX();
        else return path.timePCN( from, middle, dumpSite );
    }
    else {
        if ( ( middle == ( from + 1 ) )
             and ( to == ( middle + 1 ) ) ) return path[to].getArrivalTime() -
                         path[from].getDepartureTime();

        if ( path[to].lateArrival( path[from].getDepartureTime() + twc->TravelTime(
                                       path[from], path[middle], path[to] ) ) ) return _MAX();
        else return path.timePCN( from, middle, to );
    }
}

/*! to be used with interSw */
double Vehicle::timePCN( POS from, Trashnode &middle ) const  {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::timePCN nodes" );
    #endif
    assert ( ( from + 2 ) <= size() );

    if ( ( from + 2 ) == size() )
        if ( dumpSite.lateArrival(  path[from].getDepartureTime() + twc->TravelTime(
                                        path[from], middle, dumpSite ) ) ) return _MAX();
        else return path.timePCN( from, middle, dumpSite );
    else if ( path[from + 2].lateArrival( path[from].getDepartureTime() +
                                          twc->TravelTime( path[from], middle, path[from + 2] ) ) ) return _MAX();
    else return path.timePCN( from, middle );
}


/**

For a truck with n containers, \f$ testedMoves= n * (n +1) / 2\f$

if positive savings moves are found, those are added to moves
otherwise all the negative savings moves are added to moves

if it happens that all moves generate TWC, in that case moves does not change
if \f$ n = 0 \f$ then moves does not change

return the number of moves added to moves
*/



long int Vehicle::eval_intraSwapMoveDumps( Moves &moves, int  truckPos,
        double factor ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_intraSwapMoveDumps" );
    #endif

    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::eval_intraSwapMoveDumps";
    #endif

    if ( path.size() == 1 ) return 0;

    int fromPos, withPos;
    double newCost;
    double savings;
    double deltaTime;

    Vehicle truck = ( *this );
    std::deque<Move>  negSavingsMoves;

    double originalCost = truck.getCost();

    int originalMovesSize = moves.size();
    int deltaMovesSize = 0;
    int otherNid;
    Move move;
    double fromDelta, withDelta;


    for ( fromPos = 1; fromPos < path.size() - 1; fromPos++ ) {
        if ( isDump( fromPos ) ) continue; //skiping dump

        Trashnode node = path[fromPos]; //saved for roll back

        for ( withPos = fromPos + 1; withPos < path.size(); withPos++ ) {
            if ( isDump( withPos ) ) continue; //skiping dump

            otherNid = path[withPos].getnid();

            if ( withPos == fromPos + 1 or fromPos == withPos + 1 or withPos == fromPos - 1
                 or fromPos == withPos - 1 ) {
                if ( truck.applyMoveIntraSw( fromPos,  withPos ) ) { //move can be done
                    newCost = truck.getCost();
                    savings = originalCost - newCost;
                    truck = ( *this );
                    move.setIntraSwMove( truckPos, fromPos,  node.getnid(), withPos, otherNid,
                                         savings );

                    if ( savings > 0 ) {
                        moves.insert( move );
                        return 1;
                    }
                }
                else truck = ( *this );
            }
            else {
                fromDelta =  timePCN( fromPos - 1, withPos,
                                      fromPos + 1 ) - timePCN( fromPos - 1, fromPos, fromPos + 1 );
                withDelta =  timePCN( withPos - 1, fromPos,
                                      withPos + 1 ) - timePCN( withPos - 1, withPos, withPos + 1 );

                if ( fromDelta < 0 or withDelta < 0 ) {
                    #ifdef VRPMAXTRACE
                    DLOG( INFO ) << "timePCN( " << fromPos - 1 << "," << withPos
                                 << "," << fromPos + 1 << ")="
                                 << timePCN( fromPos - 1, withPos, fromPos + 1 );

                    DLOG( INFO ) << "timePCN( " << fromPos - 1 << "," << fromPos
                                 << "," << fromPos + 1 << ")="
                                 << timePCN( fromPos - 1, fromPos, fromPos + 1 );

                    DLOG( INFO ) << "detlaTime" << fromDelta;

                    DLOG( INFO ) << "timePCN( " << withPos - 1 << "," << fromPos
                                 << "," << withPos + 1 << ")="
                                 << timePCN( withPos - 1, fromPos, withPos + 1 );

                    DLOG( INFO ) << "timePCN( " << withPos - 1 << "," << withPos
                                 << "," << withPos + 1 << ")="
                                 << timePCN( withPos - 1, withPos, withPos + 1 );

                    DLOG( INFO ) << "deltatime " << withDelta;

                    DLOG( INFO ) << "totalDeltaTimes " << fromDelta + withDelta;
                    #endif

                    if ( truck.applyMoveIntraSw( fromPos,  withPos ) ) { //move can be done
                        newCost = truck.getCost();
                        savings = originalCost - newCost;
                        truck = ( *this );
                        move.setIntraSwMove( truckPos, fromPos,  node.getnid(), withPos, otherNid,
                                             savings );

                        if ( savings > 0 ) {
                            moves.insert( move );
                            return 1;
                        }
                    }
                    else truck = ( *this );
                }
            }
        }
    }

    return deltaMovesSize;
}


/*
    void setIntraSwMove( int fromTruck, int fromPos, int fromId, int withPos, int withId);
    what about, first try to find all positive savings
    then find one that in one truck has positive savings and the other truck has negative savings
    dont return moves where in both trucks have negative savings
*/


//does all the combinations in a 10 limit window
long int Vehicle::eval_interSwapMoveDumps( Moves &moves,
        const Vehicle &otherTruck, int  truckPos, int  otherTruckPos,  int fromPos,
        int toPos ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_interSwapMoveDumps (10 limit window)" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::eval_interSwapMoveDumps (10 limit window)";
    #endif
    assert ( fromPos < size() );
    assert ( toPos < otherTruck.size() );

    Vehicle truck = ( *this );
    Vehicle other = otherTruck;

    Trashnode tLast = path[size() - 1];
    Trashnode oLast = other.path[other.path.size() - 1];
    double truckDelta, otherDelta;
    double originalCost = truck.getCost()  + other.getCost();
    double originalDuration = truck.getDuration()  + other.getDuration();
    double newCost, savings, newDuration;
    int oldMovesSize = 0;
    int fromNodeId, toNodeId;
    Move move;

    int iLowLimit = std::max( 1, fromPos - 5 );
    int iHighLimit = std::min( size(), fromPos + 5 );
    int jLowLimit = std::max( 1, toPos - 5 );
    int jHighLimit = std::min( other.size(), toPos + 5 );

    for ( int i = iLowLimit; i < iHighLimit; i++ ) {
        assert( not ( i == 0 ) );

        if ( truck.path[i].isDump() ) continue;

        fromNodeId = truck.path[i].getnid();

        for ( int j = jLowLimit; j < jHighLimit; j++ ) {
            assert( not ( j == 0 ) );

            if ( other.path[j].isDump() ) continue;

            otherDelta = other.timePCN( j - 1, truck.path[i] ) - other.timePCN( j - 1, j,
                         j + 1 );
            truckDelta = truck.timePCN( i - 1, other.path[j] ) - truck.timePCN( i - 1, i,
                         i + 1 );

            toNodeId = other.path[j].getnid();

            if ( otherDelta < 0 or truckDelta < 0 ) {
                savings = _MIN();

                if ( truck.applyMoveInterSw( other, i, j ) ) {
                    newCost = truck.getCost() + other.getCost();
                    newDuration = truck.getDuration() + other.getDuration();
                    savings = originalCost - newCost;
                    move.setInterSwMove( truckPos,  i,  fromNodeId,  otherTruckPos, j, toNodeId,
                                         savings );
                    moves.insert( move );
                }

                truck = ( *this );
                other = otherTruck;
            }
        }
    }

    return moves.size() - oldMovesSize;
}

long int Vehicle::eval_interSwapMoveDumps( Moves &moves,
        const Vehicle &otherTruck, int  truckPos, int  otherTruckPos,
        double factor ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_interSwapMoveDumps" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::eval_interSwapMoveDumps";
    #endif
    double minSavings = -5;

    Vehicle truck = ( *this );
    Vehicle other = otherTruck;
    Trashnode tLast = path[size() - 1];
    Trashnode oLast = other.path[other.path.size() - 1];
    double truckDelta, otherDelta;
    int numNotFeasable = 0;
    double originalCost = truck.getCost()  + other.getCost();
    double originalDuration = truck.getDuration()  + other.getDuration();
    double newCost, savings, newDuration;
    int deltaMovesSize = 0;
    int fromNodeId, toNodeId;
    Move move;

    int inc = 5;

    for ( int m = 1; m < 6; m++ ) {
        for ( int i = m; i < truck.size(); i += inc ) {
            assert( not ( i == 0 ) );

            if ( truck.path[i].isDump() ) continue;

            fromNodeId = truck.path[i].getnid();

            for ( int k = 1; k < inc + 1; k++ ) {
                for ( int j = k; j < other.size(); j += inc ) {
                    assert( not ( j == 0 ) );

                    if ( other.path[j].isDump() ) continue;

                    if ( numNotFeasable > factor * ( truck.getn() * other.getn() ) ) {
                        #ifdef VRPMAXTRACE
                        DLOG( INFO ) << "LEAVING WITH numNotFeasable: " << numNotFeasable;
                        DLOG( INFO ) << "LEAVING WITH moves: " << deltaMovesSize;
                        #endif
                        return deltaMovesSize;
                    }

                    if ( deltaMovesSize > factor * ( truck.getn() * other.getn() ) ) {
                        #ifdef VRPMAXTRACE
                        DLOG( INFO ) << " LEAVING WITH moves: " << deltaMovesSize;
                        DLOG( INFO ) << " LEAVING WITH numNotFeasable: " << numNotFeasable;
                        #endif
                        return deltaMovesSize;
                    }

                    otherDelta = other.timePCN( j - 1, truck.path[i] ) - other.timePCN( j - 1, j,
                                 j + 1 );
                    truckDelta = truck.timePCN( i - 1, other.path[j] ) - truck.timePCN( i - 1, i,
                                 i + 1 );

                    /*
                    if (otherDelta < 0 or truckDelta<0) {
                    DLOG(INFO) << "otherDelta: " << otherDelta;
                    DLOG(INFO) << "truckDelta: " << truckDelta;
                    }
                    */
                    toNodeId = other.path[j].getnid();


                    savings = _MIN();

                    if ( otherDelta < 0 or truckDelta < 0 ) {
                        if ( truck.applyMoveInterSw( other, i, j ) ) {
                            newCost = truck.getCost() + other.getCost();
                            newDuration = truck.getDuration() + other.getDuration();
                            savings = originalCost - newCost;
                            move.setInterSwMove( truckPos,  i,  fromNodeId,  otherTruckPos, j, toNodeId,
                                                 savings );
                            moves.insert( move );
                            deltaMovesSize++;
                        }
                        else numNotFeasable++;

                        truck = ( *this );
                        other = otherTruck;

                        if ( savings > 0 ) deltaMovesSize += eval_interSwapMoveDumps( moves, otherTruck,
                                                                 truckPos, otherTruckPos, i, j );
                    }
                }
            }
        }
    }

    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "NORMAL WITH moves: " << deltaMovesSize;
    DLOG( INFO ) << "NORMAL WITH numNotFeasable: " << numNotFeasable;
    DLOG( INFO ) << "limit was " << ( factor * ( getn() * otherTruck.getn() ) );
    #endif

    if ( deltaMovesSize ) return deltaMovesSize;

    return 0;
}





// space reserved for TODO list
bool Vehicle::e_insertIntoFeasableTruck( const Trashnode &node, int pos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::e_insertIntoFeasableTruck" );
    #endif
    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::e_insertIntoFeasableTruck";
    #endif
    assert( feasable() );
    double localCost = cost;

    if ( not path.e__insert( node, pos, maxcapacity ) ) {
        assert( feasable() );
        return false;
    }

    evalLast();

    if ( not feasable() ) {
        path.e_remove( pos, maxcapacity );
        evalLast();
        assert( localCost == cost );
        assert( feasable() );
        return false;
    };

    assert( feasable() );

    return true;
}


/*
//dont forget, negative savings is a higher cost
bool Vehicle::eval_erase(int at, double &savings) const {
#ifdef DOSTATS
 STATS->inc("Vehicle::eval_erase");
#endif
#ifdef TESTED
DLOG(INFO) << "Entering Vehicle::eval_erase";
#endif
    assert (at<size() and at>0 );
    if ( path[at].isdump() ) { savings=_MIN(); return false;}
    Vehicle truck = (*this);
    truck.path.erase(at);
    if ( not truck.e_makeFeasable(at) ) savings = _MIN(); // -infinity
        else savings = cost - truck.cost;

    return truck.feasable();
};
*/


//dont forget, negative savings is a higher cost
bool Vehicle::eval_erase( int at, double &savings ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_erase" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::eval_erase";
    #endif
    assert ( at<size() and at>0 );

    if ( path[at].isDump() ) { savings = _MIN(); return false;}

    Vehicle truck = ( *this );
    double oldcost = truck.getCost();

    truck.path.erase( at );

    if ( not truck.e_makeFeasable( at ) ) savings = _MIN(); // -infinity
    else savings = oldcost - truck.getCost();

    #ifdef TESTED
    DLOG( INFO ) << "ERASE: oldcost: " << oldcost
                 << "\tnewcost: " << truck.getCost()
                 << "\tsavings: " << oldcost - truck.getCost();
    #endif
    return truck.feasable();
};

long int Vehicle::eval_insertMoveDumps( const Trashnode &node, Moves &moves,
                                        int fromTruck, int fromPos, int toTruck, double eraseSavings,
                                        double factor ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_insertMoveDumps" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::eval_insertMoveDumps";
    #endif
    Vehicle truck = ( *this );
    std::deque<int> unTestedPos;
    std::deque<int> unfeasablePos;
    std::deque<int> impossiblePos;
    int currentPos, testingPos;
    double oldcost = truck.getCost();
    double newcost;
    Move move;
    #ifdef TESTED
    truck.dumpCostValues();
    #endif

    for ( int i = 1; i <= size(); i++ ) unTestedPos.push_back( i );

    while ( unTestedPos.size() ) {
        currentPos = unTestedPos.back();
        unTestedPos.pop_back();
        truck.insert( node, currentPos );

        if ( not truck.e_makeFeasable( currentPos ) ) {
            #ifdef TESTED
            truck.tau();
            truck.dumpeval();
            #endif
            impossiblePos.push_back( currentPos );

            if ( path.size()*factor > impossiblePos.size() ) return moves.size();
        }
        else {
            assert ( truck.feasable() );
            newcost = truck.getCost();
            #ifdef TESTED
            DLOG( INFO ) << "insert to " << toTruck << ": oldcost" << oldcost
                         << "\tnewcost" << truck.getCost()
                         << "\ninsert savings=" << ( oldcost - newcost )
                         << "\teraseSavings=" << eraseSavings
                         << "\tsavings" << oldcost - newcost + eraseSavings;
            #endif
            move.setInsMove( fromTruck, fromPos, node.getnid(), toTruck, currentPos,
                             ( cost - truck.cost + eraseSavings )    );
            moves.insert( move );
            #ifdef TESTED
            move.dump();
            #endif
        }

        truck = ( *this );
    }

    return moves.size();
}



bool Vehicle::e_makeFeasable( int currentPos ) {
    #ifdef DOSTATS
    STATS->inc( " Vehicle::e_makeFeasable" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::e_makeFeasable";
    #endif
    path.e__adjustDumpsToMaxCapacity( currentPos, dumpSite, maxcapacity );
    evalLast();
    return feasable();
}

bool Vehicle::applyMoveINSerasePart( int nodeNid, int pos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::applyMoveINSerasePart" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::applyMoveINSerasePart";
    #endif
    // if this assertion fails might be because its not being applied
    // to the correct solution
    assert ( path[pos].getnid() == nodeNid );

    if ( not ( path[pos].getnid() == nodeNid ) )  return false;

    path.erase( pos );
    e_makeFeasable( pos );

    #ifdef DOVRPLOG
    if ( not feasable() ) dumpeval();
    #endif

    assert ( feasable() );
    return feasable();
}


bool Vehicle::applyMoveINSinsertPart( const Trashnode &node, int pos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::applyMoveINSinsertPart" );
    #endif
    path.insert( node, pos );
    e_makeFeasable( pos );

    #ifdef DOVRPLOG
    if ( not feasable() ) dumpeval();
    #endif

    assert ( feasable() );
    return feasable();
}

bool Vehicle::applyMoveInterSw( Vehicle &otherTruck, int truckPos,
                                int otherTruckPos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::applyMoveInterSw" );
    #endif

    path.swap( truckPos,  otherTruck.path, otherTruckPos );

    if ( not e_makeFeasable( truckPos ) ) return false;

    if ( not otherTruck.e_makeFeasable( otherTruckPos ) ) return false;

    //evalLast();
    //otherTruck.evalLast();

    assert ( feasable() );
    assert ( otherTruck.feasable() );
    return feasable() and otherTruck.feasable();
}

bool Vehicle::applyMoveIntraSw( int  fromPos, int withPos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::applyMoveIntraSw" );
    #endif
    path.swap( fromPos,  withPos );

    if ( not e_makeFeasable( std::min( fromPos - 1, withPos ) ) ) return false;

    //evalLast(); done in makeFeasable
    assert ( feasable() );
    return feasable() ;
}


/*
bool Vehicle::e_insertMoveDumps( const Trashnode &node, int at) {
    assert (at<=size());
//
//        path.insert(node,at);
//        path.e_moveDumps(at);
//
}
*/


// Very TIGHT insertion
// insertion will not be performed if
//      TV and CV are  generated
//  true- insertion was done
//  false- not inserted
bool Vehicle::e_insertSteadyDumpsTight( const Trashnode &node, int at ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::e_insertSteadyDumpsTight" );
    #endif
    assert ( at <= size() );


    if ( deltaCargoGeneratesCV( node, at ) ) return false;
    if ( deltaTimeGeneratesTV( node, at ) ) return false;

    #ifdef VRPMAXTRACE
    path[size() - 1].dumpeval();
    #endif

    if ( path.e_insert( node, at, maxcapacity ) ) return false;

    evalLast();

    assert ( feasable() );
    return true;
};


// end space reserved for TODO list


bool Vehicle::e_insertDumpInPath( const Trashnode &lonelyNodeAfterDump ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::e_insertDumpInPath" );
    #endif

    //we arrived here because of CV
    if ( deltaTimeGeneratesTV( dumpSite, lonelyNodeAfterDump ) ) return false;

    Trashnode dump = dumpSite;
    dump.setDemand( -getCargo() );
    path.e_push_back( dump, maxcapacity );
    path.e_push_back( lonelyNodeAfterDump, maxcapacity );
    evalLast();

    assert ( feasable() );
    return true;
};







//bool Vehicle::deltaCargoGeneratesCV_AUTO(const Trashnode &node, int pos) const { //position becomes important

bool Vehicle::deltaCargoGeneratesCV( const Trashnode &node,
                                     int pos ) const { //position becomes important
    #ifdef DOSTATS
    STATS->inc( "Vehicle::deltaCargoGeneratesCV" );
    #endif
    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::deltaCargoGeneratesCV";
    DLOG(INFO) << getCargo() << "+" << node.getDemand() << " ¿? " << getmaxcapacity();
    #endif
    //cycle until a dump is found
    int i;

    for ( i = pos; i < size() and not isDump( i ); i++ ) {};

    // two choices i points to a dump or i == size()
    // in any case the i-1 node has the truck's cargo
    #ifdef VRPMAXTRACE
    path[i - 1].dumpeval();

    DLOG( INFO ) << getCargo( i - 1 ) << "+" << node.getDemand() << " ¿? " <<
                 getmaxcapacity();

    #endif
    return  ( path[i - 1].getCargo() + node.getDemand() > maxcapacity  ) ;
};




//////////// Delta Time generates TV
bool Vehicle::deltaTimeGeneratesTV( const Trashnode &dump,
                                    const Trashnode &node ) const {
    #ifdef DOSTATS
    STATS->inc( " Vehicle::deltaTimeGeneratesTV" );
    #endif

    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::deltaTimeGeneratesTV";
    DLOG( INFO ) << " (S 1 2 3 D E )  (S 1 2 3 D N D E)"
                 << path.getDeltaTimeAfterDump( dumpSite, node ) << " + "
                 << getDuration() << " ¿? " <<  endingSite.closes();
    #endif

    return  ( path.getDeltaTimeAfterDump( dumpSite, node ) +
              getDuration()  > endingSite.closes() );
}





bool Vehicle::deltaTimeGeneratesTV( const Trashnode &node, int pos ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::deltaTimeGeneratesTV" );
    #endif
    assert( pos<=path.size() );

    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::deltaTimeGeneratesTV";

    if ( pos > path.size() )
        DLOG( INFO ) << "CANT work with this pos:" << pos;

    if ( pos==path.size() )
        DLOG(INFO) << " ( S 1 2 3 D E )  ( S 1 2 3 N D E )"
                   << path.getDeltaTime(node,dumpSite) << " + "
                   << getDuration() << " ¿ ? " <<  endingSite.closes();
    else
        DLOG(INFO) << " ( S 1 2 3 D E )  ( S 1 2 N 3 D E ) "
                   << path.getDeltaTime(node,pos) << " + "
                   << getDuration() << " ¿ ? " <<  endingSite.closes();

    endingSite.dump();
    #endif

    if ( pos==path.size() )
        return path.getDeltaTime( node, dumpSite ) +
               getDuration() > endingSite.closes();
    else
        return path.getDeltaTime( node, pos ) +
               getDuration() > endingSite.closes();
}

