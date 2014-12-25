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



long int Vehicle::eval_intraSwapMoveDumps( Moves &moves, POS  truckPos) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_intraSwapMoveDumps" );
    #endif

    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::eval_intraSwapMoveDumps";
    #endif

    if ( path.size() == 1 ) return 0;

    POS fromPos, withPos;
    double newCost;
    double savings;
    //double deltaTime;

    Vehicle truck = ( *this );
    std::deque<Move>  negSavingsMoves;

    double originalCost = truck.getCost();

    //int originalMovesSize = moves.size();
    int deltaMovesSize = 0;
    UID otherNid;
    Move move;
    double fromDelta, withDelta;


    for ( fromPos = 1; fromPos < path.size() - 1; fromPos++ ) {
        if ( isDump( fromPos ) ) continue; //skiping dump

        Trashnode node = path[fromPos]; //saved for roll back

        for ( withPos = fromPos + 1; withPos < path.size(); withPos++ ) {
            if ( isDump( withPos ) ) continue; //skiping dump

            otherNid = path[withPos].nid();

            if ( withPos == fromPos + 1 or fromPos == withPos + 1 or withPos == fromPos - 1
                 or fromPos == withPos - 1 ) {
                if ( truck.applyMoveIntraSw( fromPos,  withPos ) ) { //move can be done
                    newCost = truck.getCost();
                    savings = originalCost - newCost;
                    truck = ( *this );
                    move.setIntraSwMove( truckPos, fromPos,  node.nid(), withPos, otherNid,
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
                        move.setIntraSwMove( truckPos, fromPos,  node.nid(), withPos, otherNid,
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
        const Vehicle &otherTruck, POS  truckPos, POS  otherTruckPos,  POS fromPos,
        POS toPos ) const {
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
    //double originalDuration = truck.getDuration()  + other.getDuration();
    double newCost, savings;
    //double newDuration;
    int oldMovesSize = 0;
    int fromNodeId, toNodeId;
    Move move;

    int iLowLimit = std::max( POS(1), fromPos - 5 );
    int iHighLimit = std::min( size(), fromPos + 5 );
    int jLowLimit = std::max( POS(1), toPos - 5 );
    int jHighLimit = std::min( other.size(), toPos + 5 );

    for ( int i = iLowLimit; i < iHighLimit; i++ ) {
        assert( not ( i == 0 ) );

        if ( truck.path[i].isDump() ) continue;

        fromNodeId = truck.path[i].nid();

        for ( int j = jLowLimit; j < jHighLimit; j++ ) {
            assert( not ( j == 0 ) );

            if ( other.path[j].isDump() ) continue;

            otherDelta = other.timePCN( j - 1, truck.path[i] ) - other.timePCN( j - 1, j,
                         j + 1 );
            truckDelta = truck.timePCN( i - 1, other.path[j] ) - truck.timePCN( i - 1, i,
                         i + 1 );

            toNodeId = other.path[j].nid();

            if ( otherDelta < 0 or truckDelta < 0 ) {
                savings = _MIN();

                if ( truck.applyMoveInterSw( other, i, j ) ) {
                    newCost = truck.getCost() + other.getCost();
                    //newDuration = truck.getDuration() + other.getDuration();
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
        const Vehicle &otherTruck, POS  truckPos, POS  otherTruckPos,
        double factor ) const {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::eval_interSwapMoveDumps" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::eval_interSwapMoveDumps";
    #endif
    //double minSavings = -5;

    Vehicle truck = ( *this );
    Vehicle other = otherTruck;
    Trashnode tLast = path[size() - 1];
    Trashnode oLast = other.path[other.path.size() - 1];
    double truckDelta, otherDelta;
    int numNotFeasable = 0;
    double originalCost = truck.getCost()  + other.getCost();
    //double originalDuration = truck.getDuration()  + other.getDuration();
    double newCost, savings;
    //double newDuration;
    int deltaMovesSize = 0;
    int fromNodeId, toNodeId;
    Move move;

    UINT inc = 5;

    for ( UINT m = 1; m < 6; m++ ) {
        for ( UINT i = m; i < truck.size(); i += inc ) {
            assert( not ( i == 0 ) );

            if ( truck.path[i].isDump() ) continue;

            fromNodeId = truck.path[i].nid();

            for ( UINT k = 1; k < inc + 1; k++ ) {
                for ( UINT j = k; j < other.size(); j += inc ) {
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
                    toNodeId = other.path[j].nid();


                    savings = _MIN();

                    if ( otherDelta < 0 or truckDelta < 0 ) {
                        if ( truck.applyMoveInterSw( other, i, j ) ) {
                            newCost = truck.getCost() + other.getCost();
                            //newDuration = truck.getDuration() + other.getDuration();
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
bool Vehicle::e_insertIntoFeasableTruck( const Trashnode &node, POS pos ) {
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
        path.e_erase( pos, maxcapacity );
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
bool Vehicle::eval_erase( POS at, double &savings ) const {
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
                                        POS fromTruck, POS fromPos, POS toTruck, 
                                        double eraseSavings) const {
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
    POS currentPos;
    Move move;

    //POS testingPos;
    #ifdef TESTED
    double oldcost = truck.getCost();
    double newcost;
    truck.dumpCostValues();
    #endif

    for ( UINT i = 1; i <= size(); i++ ) unTestedPos.push_back( i );

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

            //if ( path.size()*factor > impossiblePos.size() ) return moves.size();
        }
        else {
            assert ( truck.feasable() );
            #ifdef TESTED
            newcost = truck.getCost();
            DLOG( INFO ) << "insert to " << toTruck << ": oldcost" << oldcost
                         << "\tnewcost" << truck.getCost()
                         << "\ninsert savings=" << ( oldcost - newcost )
                         << "\teraseSavings=" << eraseSavings
                         << "\tsavings" << oldcost - newcost + eraseSavings;
            #endif
            move.setInsMove( fromTruck, fromPos, node.nid(), toTruck, currentPos,
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



bool Vehicle::e_makeFeasable( POS currentPos ) {
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

bool Vehicle::applyMoveINSerasePart( UID nodeNid, POS pos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::applyMoveINSerasePart" );
    #endif
    #ifdef TESTED
    DLOG( INFO ) << "Entering Vehicle::applyMoveINSerasePart";
    #endif
    // if this assertion fails might be because its not being applied
    // to the correct solution
    assert ( path[pos].nid() == nodeNid );

    if ( not ( path[pos].nid() == nodeNid ) )  return false;

    path.erase( pos );
    e_makeFeasable( pos );

    #ifdef DOVRPLOG
    if ( not feasable() ) dumpeval();
    #endif

    assert ( feasable() );
    return feasable();
}


bool Vehicle::applyMoveINSinsertPart( const Trashnode &node, POS pos ) {
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

bool Vehicle::applyMoveInterSw( Vehicle &otherTruck, POS truckPos,
                                POS otherTruckPos ) {
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

bool Vehicle::applyMoveIntraSw( POS  fromPos, POS withPos ) {
    #ifdef DOSTATS
    STATS->inc( "Vehicle::applyMoveIntraSw" );
    #endif
    path.swap( fromPos,  withPos );

    if ( not e_makeFeasable( std::min( fromPos - 1, withPos ) ) ) return false;

    //evalLast(); done in makeFeasable
    assert ( feasable() );
    return feasable() ;
}


#if 0
bool Vehicle::e_insertMoveDumps( const Trashnode &node, int at) {
    assert (at<=size());
    path.insert(node,at);
    path.e_moveDumps(at);
}
#endif

#if 0
// Very TIGHT insertion
// insertion will not be performed if
//      TV and CV are  generated
//  true- insertion was done
//  false- not inserted
bool Vehicle::e_insertSteadyDumpsTight( const Trashnode &node, POS at ) {
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
                                     POS pos ) const { //position becomes important
    #ifdef DOSTATS
    STATS->inc( "Vehicle::deltaCargoGeneratesCV" );
    #endif
    #ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering Vehicle::deltaCargoGeneratesCV";
    DLOG(INFO) << getCargo() << "+" << node.getDemand() << " ¿? " << getmaxcapacity();
    #endif
    //cycle until a dump is found
    UINT i;

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
                 << path.getDeltaTimeAfterDump( dump, node ) << " + "
                 << getDuration() << " ¿? " <<  endingSite.closes();
    #endif

    return  ( path.getDeltaTimeAfterDump( dump, node ) +
              getDuration()  > endingSite.closes() );
}





bool Vehicle::deltaTimeGeneratesTV( const Trashnode &node, POS pos ) const {
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
#endif

void Vehicle::setInitialValues( const Trashnode &node, const Bucket &picks ) {

        C = node;
        ttSC = twc->getAverageTime( depot, picks );
        ttDC = twc->getAverageTime( dumpSite, picks );
        ttCD = twc->getAverageTime( picks, dumpSite );
        ttDE = twc->TravelTime( dumpSite, endingSite );
        ttCC = twc->TravelTime( C, C );
        serviceE = endingSite.serviceTime();
        shiftLength = endTime - startTime;
        e_makeFeasable( 0 );
        Z = floor( maxcapacity / C.demand() );
        arrivalEclosesLast = C.closes() + ttCD + dumpSite.serviceTime() + ttDE;
        totalTime = 0;

        N = -1;

        do {
            N++;
            totalTime = depot.serviceTime()  + ttSC + ttDE + endingSite.serviceTime()
                        + N * Z * C.serviceTime() + N * ( Z - 1 ) * ttCC + ( N - 1 ) * ttDC
                        + N * ( dumpSite.serviceTime() + ttCD );
        }
        while ( totalTime < arrivalEclosesLast + serviceE );

        forcedWaitTime = endTime - ( arrivalEclosesLast  +  serviceE );
        totalWaitTime = endTime - ( endingSite.getArrivalTime() + serviceE );
        idleTimeSCDE = C.closes() - ( depot.serviceTime() + ttSC );
        z1 = idleTimeSCDE / ( C.serviceTime() + ttCC );
        idleTimeSDCDE = C.closes() - ( dumpSite.getDepartureTime() + ttDC );
        z2 = idleTimeSDCDE / ( C.serviceTime() + ttCC );
        idleTime = totalWaitTime - forcedWaitTime;
    };

void Vehicle::setCost() {
        last = ( size() > 1 ) ? path[size() - 1] : C ;
        realttSC = path.size() > 1 ? path[1].getTotTravelTime()  : ttSC;
        ttSC = std::min( realttSC, ttSC );

        realttCC = size() > 1 ? ( path.getTotTravelTime() - realttSC ) /
                   ( size() - 1 ) : ttCC;
        ttCC = std::min( realttCC, ttCC );

        realttCD = 0;
        realttDC = 0;
        double realZ = 0;

        if ( path.getDumpVisits() ) {
            for ( UINT i = 1; i < path.size() - 1; i++ ) {
                realZ++;

                if ( path[i - 1].isDump() )
                    realttCD += twc->TravelTime( path[i - 1], path[i] );

                if ( path[i].isDump() )
                    realttDC += twc->TravelTime( path[i], path[i + 1] );
            }
        }
        else realttDC = ttDC;

        realttCD = ( realttCD + twc->TravelTime( last,
                     dumpSite ) ) / ( path.getDumpVisits() + 1.0 );

        ttCD = std::min( realttCD, ttCD );
        ttDC = std::min( realttDC, ttDC );

        realttDE = ttDE;

        realArrivalEclosesLast = last.closes() +  last.serviceTime() +
                                 twc->TravelTime( last, dumpSite ) +
                                 dumpSite.serviceTime() + realttDE;
        //>0 the latest the truck can arrive
        arrivalEclosesLast = std::max( realArrivalEclosesLast,
                                       arrivalEclosesLast );

        realForcedWaitTime = endTime - ( realArrivalEclosesLast  +  serviceE );
        forcedWaitTime = std::min ( realForcedWaitTime , forcedWaitTime );

        n  = size() - 1 - ( realN() - 1 );
        //>0 allways good, we have one more container (truck point fo view)
        double deltan = n - lastn;
        //setting this n as the last
        lastn = n;

        z = ( realN() == 1 ) ?  n  : n % Z ;
        //>0 good, we can work more containers/trip
        //double deltaZ = Z - z;
        Z = std::max( Z, z );

        // ==0 we are in the limit of container pickup
        // >0 we need to pickup more containers
        Zmissing = Z - z;

        //its never negative
        assert( Zmissing >= 0 );

        realTotalTime = endingSite.getArrivalTime();
        lastRealTotalTime = realTotalTime;

	#ifdef DOVRPLOG
        if ( realArrivalEclosesLast < realTotalTime ) {
            last.dumpeval(maxcapacity);
            dumpCostValues();
        };
	#endif

        //otherwise we are in a TWV and something is wrong on the calculation
        //assert ( realArrivalEclosesLast > realTotalTime );

        realIdleTime =  realArrivalEclosesLast -  realTotalTime ;

        realIdleTimeSCDE =  ( Zmissing > 0 ) ?
                            ( C.serviceTime() + realttCC ) * Zmissing :
                            C.closes() - ( depot.getDepartureTime() +  realttSC ) ;

        realz1 = std::min( ( int ) ( floor( realIdleTime /
                ( C.serviceTime() + realttCC ) ) ) , Zmissing ) ;

        //cant have negative idleTime
        realIdleTimeSDCDE = std::max( ( C.closes() -
                ( dumpSite.getDepartureTime() + realttDC ) ) , 0.0 );

        realz2 = floor( realIdleTimeSDCDE / ( C.serviceTime() +  realttCC ) );

        sumIdle = realIdleTimeSCDE + realIdleTimeSDCDE + realIdleTime;

        //tengo z contenedores en el utimo viaje
        //me faltan Zmissing contenedores para un viaje lleno al dump
        //pero solo puedo hacer z1 contenedores mas en ese viaje al dump;


        // aumente el numero de contenedores  deltaz1>0 es bueno,
        // aumente contenedor y pudo puedo aumentar mas contenedores todavia
        // (no tiene sentido)
        if ( deltan >= 0 )
            z1 = std::max ( z1 - 1, realz1 );

        //el numero de contenedores no cambio  deltaz1>0 es bueno
        if ( deltan == 0 )
            z1 = std::max ( z1, realz1 ) ;

        // quite un contenedor, deltaz>0 me hace falta un contenedor z1 debe
        // de haber aumentado minimo en 1
        if ( deltan < 0 )
            z1 = std::max ( z1 + 1, realz1 );



        // aumente el numero de contenedores  deltaz2>0 es bueno,
        // aumente contenedor y pudo puedo aumentar mas contenedores todavia
        // (no tiene sentido)
        if ( deltan >= 0 )
            z2 = std::max ( z2 - 1, realz2 );

        // el numero de contenedores no cambio  deltaz2>0 es bueno
        if ( deltan == 0 )
            z2 = std::max ( z2, realz2 ) ;

        // quite un contenedor, deltaz>0 me hace falta un contenedor z1 debe
        // de haber aumentado minimo en 1
        if ( deltan < 0 )
            z2 = std::max ( z2 + 1, realz2 );

        #ifdef VRPMAXTRACE
        // >0 el promedio de viaje entre contenedores es mayor
        double deltattCC = realttCC - ttCC;
        // >0 viaja mas lejos para llegar al primer contenedor
        double deltattSC = realttSC - ttSC;
        //>0 bad thing the forcedWaitTime has increased
        double deltaForcedWaitTime = realForcedWaitTime - forcedWaitTime;
        //>0 the latest the truck can arrive is better
        double deltaArrivalEclosesLast = realArrivalEclosesLast -
                                         arrivalEclosesLast;
        // >0 el viaje del dump al contenedor es mas largo que
        // lo esperado (worse)
        double deltattDC = realttDC - ttDC;
        // >0 el viaje del contenedor al dump es mar largo que lo esperado
        double deltattCD = realttCD - ttCD;
        double deltaz1 = realz1 - z1;
        // >0 the total time has increased  good or bad depends on deltan
        double deltaRealTotalTime = realTotalTime - lastRealTotalTime;
        double deltaz2 = realz2 - z2;
        DLOG( INFO ) << "TODOS LOS DELTAS2\n"
                     << "deltattSC    " << deltattSC    << "\n"
                     << "deltattCC    " << deltattCC    << "\n"
                     << "deltattDC    " << deltattDC    << "\n"
                     << "deltattCD    " << deltattCD    << "\n"
                     << "deltaArrivalEclosesLast    " << deltaArrivalEclosesLast    << "\n"
                     << "deltaForcedWaitTime    " << deltaForcedWaitTime    << "\n"
                     << "deltan    " << deltan    << "\n"
                     << "deltaZ    " << deltaZ    << "\n"
                     << "deltaRealTotalTime    " << deltaRealTotalTime    << "\n"
                     << "deltaz1    " << deltaz1    << "\n"
                     << "deltaz2    " << deltaz2    << "\n";
        #endif


        //workNotDonePerc = ( double ( realz1 + realz2 ) )  / ( double ( double(
        //                      n ) + double( realz1 ) + double( realz2 ) ) );
        //double workDonePerc = 1 - workNotDonePerc;
        // v_cost =  realTotalTime * (1 + workNotDonePerc) + sumIdle * ( 1 + workDonePerc) + getDuration();
        v_cost = getDuration();
    };

double Vehicle::getDeltaCost( double deltaTravelTime, int deltan ) {
        double newrealTotalTime = realTotalTime + deltaTravelTime;
        double newrealIdleTime = realArrivalEclosesLast - newrealTotalTime;
        int newn = n + deltan;
        int newz = ( realN() == 1 ) ?  newn  : newn % Z ;
        int newZmissing = ( Z > newz ) ? Z - newz : 0;
        double newrealIdleTimeSCDE =  ( newz ) ? newrealIdleTime -
                                      ( C.serviceTime() + realttCC ) * newZmissing :
                                      C.closes() - ( depot.getDepartureTime() +  realttSC );
        double newrealz1 = std::min ( ( int ) ( floor( newrealIdleTime /
                                                ( C.serviceTime() + realttCC ) ) ) , newZmissing ) ;
        double newrealIdleTimeSDCDE =  C.closes() - ( dumpSite.getDepartureTime() +
                                       deltaTravelTime + realttDC );
        double  newrealz2 = newrealIdleTimeSDCDE / ( C.serviceTime() +  realttCC );

        double newv_cost = newrealTotalTime + ( newrealz1 + newrealz2 ) * newn +
                           newrealIdleTimeSCDE + newrealIdleTimeSDCDE;
        double deltacost = newv_cost - v_cost;
        return deltacost;
    };




    #ifdef DOVRPLOG
void Vehicle::dumpCostValues() const {
        DLOG( INFO ) << " +++++++++++++++++++++  	 TRUCK #<<" << vid
                     << "      +++++++++++++++++++++";
        DLOG( INFO ) << " Average Container:";
        C.dump();
        DLOG( INFO ) << " ------  current path -------";
        tau();

        DLOG( INFO ) << " ------  truck time limits -------";
        DLOG( INFO ) << "Shift Starts\t" << startTime;
        DLOG( INFO ) << "Shift ends\t" << endTime;
        DLOG( INFO ) << "Shift length\t" << shiftLength;


        DLOG( INFO ) << "------Real  Values of current truck in the solution -------\n"
                     << "                   realttSC\t" << realttSC << "\n"
                     << "                   realttCC\t" << realttCC << "\n"
                     << "                   realttCD\t" << realttCD << "\n"
                     << "                   realttDC\t" << realttDC << "\n"
                     << "                   realttDE\t" << realttDE << "\n"
                     << "                 service(E)\t" << serviceE << "\n"
                     << "                maxcapacity\t" << maxcapacity << "\n"
                     << "              C.getdemand()\t" << C.demand() << "\n"
                     << "         C.getservicetime()\t" << C.serviceTime()  << "\n"
                     << "                 C.closes()\t" << C.closes() << "\n"
                     << "    path[size()-1].closes()\t" << path[size() - 1].closes() << "\n"
                     << "  dumpSite.getservicetime()\t" << dumpSite.serviceTime()  << "\n"
                     << "dumpSite.getDepartureTime()\t" << dumpSite.getDepartureTime() << "\n"
                     << "                      realN\t" << realN()  << "\n"
                     << "endingSite.getArrivalTime()\t" << endingSite.getArrivalTime()  << "\n"
                     << "   depot.getDepartureTime()\t" << depot.getDepartureTime() << "\n"
                     << "                       size\t" << size() << "\n"
                     //<<" \t"<<  <<"\n"
                     //<<" \t"<<  <<"\n"
                     //<<" \t"<<  <<"\n"

                     << "                     Z =\t" << Z  <<
                     "\t= floor( maxcapacity/C.getdemand() )\t" << Z << "\n"
                     << "                     n =\t" << n  << "\t=size() - 1 - ( realN()-1 )  \t" <<
                     n << "\n"
                     << "                     z =\t" << z << "\t=(realN()==1)  z = n  : n % Z\t"  <<
                     z << "\n"
                     << "              Zmissing =\t" << Zmissing << "\t=Z-z\t" << Zmissing << "\n"
                     << "                realz1 =\t" << realz1 <<
                     "\t== min ( floor ( realIdleTimeSCDE / (C.getservicetime() + realttCC) ) , Zmissing )\t"
                     << realz1 << "\n"
                     << "                realz2 =\t" << realz2 <<
                     "\t=idleTimeSDCDE / (C.getservicetime() + realttCC)\t"    << realz2 << "\n"

                     << "realArrivalEclosesLast =\t" << realArrivalEclosesLast <<
                     "\t=path[size()-1].closes() + realttCD + dumpSite.getservicetime() + realttDE \t"
                     << realArrivalEclosesLast << " \n "
                     << "    realForcedWaitTime =\t" << realForcedWaitTime  <<
                     "\t=shiftEnds -( realArrivalEclosesLast  +  serviceE )\t" << realForcedWaitTime
                     << "\n"
                     << "         realTotalTime =\t" << realTotalTime  <<
                     "\t=endingSite.getArrivalTime()\t" << realTotalTime << "\n"


                     << "          realIdleTime =\t" << realIdleTime <<
                     "\t=realArrivalEclosesLast -  realTotalTime\t" << realIdleTime << "\n"
                     << "      realIdleTimeSCDE =\t" << realIdleTimeSCDE  <<
                     "\t=( Zmissing>0 )?  (C.getservicetime() + realttCC ) * Zmissing :\n"
                     "\t\t(Zmissing==0? C.closes() - ( depot.getDepartureTime() +  realttSC):0) ;\t"
                     << realIdleTimeSCDE << "\n"
                     << "     realIdleTimeSDCDE =\t" << realIdleTimeSDCDE  <<
                     "\t=C.closes() - ( dumpSite.getDepartureTime() + realttDC)\t" <<
                     realIdleTimeSDCDE << "\n"

                     << "                sumIdle=\t" << sumIdle <<
                     "\t=sumIdle=realIdleTimeSCDE+realIdleTimeSDCDE+realIdleTime\t" << sumIdle <<
                     "\n"

                     << "        workNotDonePerc=\t" << workNotDonePerc <<
                     "\t=(double (realz1 + realz2))  /(double (n + realz1+realz2))\n"
                     << "     1+ workNotDonePerc=\t" << ( 1 + workNotDonePerc ) <<
                     "\t=(double (realz1 + realz2))  /(double (n + realz1+realz2))\n"
                     << "realTotalTime + sumIdle)=\t" << ( realTotalTime + sumIdle ) << "\n"
                     << "\n\n             v_cost=\t" << v_cost <<
                     "\t= (realTotalTime + sumIdle) *( 1 + workNotDonePerc)\n";

        
	#if 0
                <<"\n\n\n DELTA TIME SIMULATION\n"
                <<"if a container is added into a very full truck:\n";

                for (double delta=-20; delta<20;delta++) { //changes in time
                    if (n) {
                        DLOG(INFO) <<"same amount of containers delta="<<delta<<  "\t    delta+delta/n=" <<(penalty=delta/n)<<"\t";
                        DLOG(INFO) <<"penalty*sumIdle= "<<(penalty*sumIdle)<<"\n";
                    }
                    if (n+1){
                         DLOG(INFO) <<"1 container more          delta="<<delta<<"\tdelta+delta/(n+1)="<<(delta/(n+1))<<"\t";
                        DLOG(INFO) <<"penalty*sumIdle= "<<(penalty*sumIdle)<<"\n";
                    }
                    if (n-1) {
                         DLOG(INFO) <<"1 container less          delta ="<<delta<<"\tdelta+delta/(n-1)="<<(delta/(n-1))<<"\t";
                        DLOG(INFO) <<"penalty*sumIdle= "<<(penalty*sumIdle)<<"\n";
                    }
                }
        #endif

	#if 0
                DLOG(INFO) <<"\n\n\n ------estimated  Values for emtpy truck that is in the solution -------\n"
                <<"ttSC=\t" <<ttSC<<"\n"
                <<"ttCC=\t" <<ttCC<<"\n"
                <<"ttCD=\t" <<ttCD<<"\n"
                <<"ttDC=\t" <<ttDC<<"\n"
                <<"ttDE=\t" <<ttDE<<"\n"
                        <<"service(E)\t"<<serviceE<<"\n"
                        <<" Z = floor( maxcapacity/C.getdemand() )    <<--- this is still the estimation\n"
                        <<Z<<" = floor( "<<maxcapacity<<"/"<<C.getdemand()<<" )\n"

                <<" \narrivalEcloseslast = C.closes() + ttCD + dumpSite.getservicetime() + ttDE \n"
                << arrivalEcloseslast<< " = " <<C.closes()<<" + "<<ttCD<<" + "<<dumpSite.getservicetime()<<" + "<<ttDE<< "\n"

                <<"\n forcedWaitTime = shiftEnds -( arrivalEcloseslast  +  serviceE )\n"
                <<forcedWaitTime<<" = "<<endTime<<" - (" <<arrivalEcloseslast <<" + " <<serviceE<<" )\n"

                        <<" \nwith N=1\n"
                        <<" upperLimit(totalTime) = depot.getservicetime()  + ttSC + ttDE + endingSite.getservicetime()\n"
                                <<"+ N * Z * C.getservicetime() + N * (Z - 1) * ttCC + (N -1) * ttDC\n"
                                <<"+ N * ( dumpSite.getservicetime() + ttCD )\n"
                        << totalTime<<" = "<< depot.getservicetime()<< " + "<< ttSC<<" + "<<ttDE<<" + "<< endingSite.getservicetime()<<"\n"
                                <<" + "<< N<<" *"<< Z<< " * "<< C.getservicetime()<<" +"<< N<<" * ("<<Z<<" - 1) * "<<ttCC<<" + ("<<N<<" -1) *"<< ttDC<<"\n"
                                <<" + "<< N<<" * (" <<dumpSite.getservicetime()<<" +"<< ttCD<<" )"<<"\n"

                        <<" \n last (and only trip) can/cant serve Z containers? =  upperLimit(totalTime) <= arrivalEcloseslast (CAN) \n"
                        <<" last (and only trip) " <<( totalTime <= arrivalEcloseslast ?"CAN":"CAN NOT" )<< " serve <<"<<Z<<" containers  "
                                << (totalTime <= arrivalEcloseslast) <<"="<<totalTime<<" <= " <<arrivalEcloseslast <<"\n"

                        <<" \n idleTimeSCDE = C.closes() - ( depot.getDepartureTime() + ttSC )\n"
                        <<idleTimeSCDE<<" = "<< C.closes()<<" - ( "<<depot.getDepartureTime()<<" + "<< ttSC<<" )\n"

                        <<"\n z1 = idleTimeSCDE / (C.getservicetime() + ttCC)\n"
                        <<z1<<" = " <<idleTimeSCDE<<" / ( "<<C.getservicetime()<<" + "<< ttCC<<" )\n"
                        <<z1<<" containers can be served in a trip: SCDE\n"

                        <<" \n idleTimeSDCDE = C.closes() - ( dumpSite.getDepartureTime() + ttDC)\n"
                        <<idleTimeSDCDE<<" = "<< C.closes()<<" - ( "<<dumpSite.getDepartureTime()<<" + "<< ttDC<<" )\n"

                        <<"\n z2 = idleTimeSDCDE / (C.getservicetime() + ttCC)\n"
                        <<z2<<" = " <<idleTimeSDCDE<<" / ( "<<C.getservicetime()<<" + "<< ttCC<<" )\n"
                        <<z2<<" containers can be served in a trip: SDCDE\n"




        ;
                DLOG(INFO) <<"\n\n\n ------  DOCUMENT COST  VARIABLES -------" <<" ------  REAL COST  VARIABLES -------\t "<<" ------  PERCENTAGES  -------\n"
                    <<"ttSC=\t" <<ttSC<<"\t" <<"realttSC=\t"    <<realttSC<<"\t" <<"realttSC/ttSC=\t\t" <<realttSC/ttSC*100<<"%\n"
                    <<"ttCC=\t" <<ttCC<<"\t" <<"realttCC=\t"    <<realttCC<<"\t" <<"realttCC/ttCC=\t\t" <<realttCC/ttCC*100<<"%\n"
                    <<"ttCD=\t" <<ttCD<<"\t" <<"realttCD=\t"    <<realttCD<<"\t" <<"realttCD/ttCD=\t\t" <<realttCD/ttCD*100<<"%\n"
                    <<"ttDC=\t" <<ttDC<<"\t" <<"realttDC=\t"    <<realttDC<<"\t" <<"realttDC/ttDC=\t\t" <<realttDC/ttDC*100<<"%\n"
                    <<"ttDE=\t" <<ttDE<<"\t" <<"realttDE=\t"    <<realttDE<<"\t" <<"realttDE/ttDE=\t\t" <<realttDE/ttDE*100<<"%\n"
                    <<"service(E)\t"<<serviceE<<"\t" <<"service(E)\t"<<serviceE<<"\n"
                    <<"Z\t\t"   <<Z<<"\n"
                    <<"N\t\t"   <<N<<"\t" <<"real N\t"<<realN()<<"\t" <<"realN/N \t\t"<<realN()/N*100<<"%\n"
                    <<"totalTime\t" <<totalTime<<"\t" <<"realTotalTime\t"<<realTotalTime<<"\t" <<"realTotalTime/totalTime\t"<<realTotalTime/totalTime*100<<"%\n"
                                                                             <<"\t\t\t\t\t\t\t"<<"realTotalTime/shiftLength\t"<<realTotalTime/shiftLength*100<<"%\n"
                    <<"totalWaitTime\t"<<totalWaitTime<<"\t\t\t\ttotalWaitTime/totalTime\t"<<totalWaitTime/totalTime*100<<"%\n"
                                                <<"\t\t\t\t\t\t\t"<<"totalWaitTime/shiftLength\t"<<totalWaitTime/shiftLength*100<<"%\n"

                    <<"forcedWaitTime\t"<<forcedWaitTime<<"\t\t\t\t" <<"forcedWaitTime/shiftLength\t"<<forcedWaitTime/shiftLength*100<<"%\n"
                    <<"idleTime\t"<<idleTime<<"\t"<<"idleTime/shiftLength\t"<<idleTime/shiftLength*100<<"%\n"
                    <<"arrivalEcloseslast\t"<<arrivalEcloseslast<<"\n";
       #endif 
    };
   #endif
