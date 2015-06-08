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
double Vehicle1::timePCN( POS from, POS middle, POS to ) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::timePCN positions" );
#endif
  assert(from < path.size());
  assert(middle < path.size());
  assert(to <= path.size());

  if ( to == size() ) {
    if ( ( middle == ( from + 1 ) )
         and ( to == ( middle + 1 ) ) ) return dumpSite.arrivalTime() -
               path[from].departureTime();

    if ( dumpSite.lateArrival(path[from].departureTime()
                              + twc->TravelTime(path[from], path[middle], dumpSite)))
      return VRP_MAX();
    else
      return path.timePCN(from, middle, dumpSite);
  } else {
    if ( (middle == (from + 1) ) and (to == (middle + 1)))
      return path[to].arrivalTime() - path[from].departureTime();

    if ( path[to].lateArrival(path[from].departureTime()
                              + twc->TravelTime(path[from], path[middle], path[to])) )
      return VRP_MAX();
    else
      return path.timePCN( from, middle, to );
  }
}

/*! to be used with interSw */
double Vehicle1::timePCN( POS from, Trashnode &middle ) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::timePCN nodes" );
#endif
  assert(from < path.size());
  assert ((from + 2) <= size());

  if ( (from + 2) == size() )
    if ( dumpSite.lateArrival(path[from].departureTime()
                              + twc->TravelTime(path[from], middle, dumpSite)) )
      return VRP_MAX();
    else
      return path.timePCN(from, middle, dumpSite);
  else if ( path[from + 2].lateArrival( path[from].departureTime() +
                                        twc->TravelTime(path[from], middle, path[from + 2])) )
    return VRP_MAX();
  else
    return path.timePCN( from, middle );
}

#if 0
// the truck treated as a trip
void Vehicle1::intraTripOptimizationNoOsrm() {
  bool oldStateOsrm = osrmi->getUse();
  osrmi->useOsrm(false);

  // POS fromPos, withPos;
  Vehicle1 trip = (*this);
  trip.push_back(getDumpSite());
#if 0
  Bucket inPath, notInPath;
  for (UINT i = 1; i < trip.size()-1; ++i) {
    if (twc->isInPath(trip[i-1], trip[i], trip[i+1])) {
      inPath.push_back(trip[i]);
    } else {
      notInPath.push_back(trip[i]);
    };
  }
  inPath.dumpid("inPath");
  notInPath.dumpid("notInPath");
#endif

  double removeTime, insertTime, deltaTime, bestDelta;
  for (int pos = trip.size()-2; pos > 0; --pos) { 
    // DLOG(INFO) << "POS = " << pos;
    UINT from = trip[pos-1].nid();
    UINT middle = trip[pos].nid();
    UINT to = trip[pos+1].nid();
    Trashnode node = trip[pos];
    bestDelta = 9999999;
    int bestI=pos-1;

    removeTime = twc->TravelTime(from, to) - twc->TravelTime(from, middle, to);
    // DLOG(INFO) << trip[pos-1].id() << " " << node.id() << " " << trip[pos+1].id() << " from " << twc->TravelTime(from, middle, to);
    // DLOG(INFO) << trip[pos].id() << " " << trip[pos+1].id() << "to" << twc->TravelTime(from, to);

    // trip.tau("before erasing the last node");
    trip.e_remove(pos);
    // trip.tau("after erasing the last node");
    for (UINT i = 0; i < trip.size()-2; ++i) {
      UINT i_nid = trip[i].nid();
      UINT j_nid = trip[i+1].nid();

      insertTime = twc->TravelTime(i_nid, middle, j_nid) - twc->TravelTime(i_nid, j_nid);
      // DLOG(INFO) << trip[i].id() << " " << trip[i+1].id() << " from " << twc->TravelTime(i_nid, j_nid);
      // DLOG(INFO) << trip[i].id() << " " << node.id() << " " << trip[i+1].id() << "  to" << twc->TravelTime(i_nid, middle, j_nid);
      // DLOG(INFO) << "remove " <<  removeTime << " insert: " << insertTime << " delta: "<< removeTime+insertTime;
      deltaTime = removeTime + insertTime;
      if (deltaTime < bestDelta) {
        bestDelta = deltaTime;
        bestI = i;
      }
    }  // for i

    // DLOG(INFO) << "inserting after " << trip[bestI].id();
    trip.e_insert(node, bestI+1);
    // trip.tau("after moving the last node");
    if (bestI == pos+1) pos = trip.size()-1;
  
  }  // for pos

// assert(true==false);
  trip.tau("before deleteing dump");
  trip.e_remove(trip.size()-1);
  trip.tau("after deleteing dump");
  (*this) = trip;
  osrmi->useOsrm(oldStateOsrm);
}
#endif

/**

For a truck with n containers, \f$ testedMoves= n * (n +1) / 2\f$

if positive savings moves are found, those are added to moves
otherwise all the negative savings moves are added to moves

if it happens that all moves generate TWC, in that case moves does not change
if \f$ n = 0 \f$ then moves does not change

return the number of moves added to moves
*/




long int Vehicle1::eval_intraSwapMoveDumps(Moves &moves, POS  truckPos) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::eval_intraSwapMoveDumps" );
#endif

#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering Vehicle1::eval_intraSwapMoveDumps";
#endif

  if ( path.size() == 1 ) return 0;

  POS fromPos, withPos;
  double newCost;
  double savings;
  //double deltaTime;

  Vehicle1 truck = ( *this );
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
        } else truck = ( *this );
      } else {
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
          } else truck = ( *this );
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
long int Vehicle1::eval_interSwapMoveDumps( Moves &moves,
    const Vehicle1 &otherTruck, POS  truckPos, POS  otherTruckPos,  POS fromPos,
    POS toPos ) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::eval_interSwapMoveDumps (10 limit window)" );
#endif
#ifdef TESTED
  DLOG( INFO ) << "Entering Vehicle1::eval_interSwapMoveDumps (10 limit window)";
#endif
  assert ( fromPos < size() );
  assert ( toPos < otherTruck.size() );

  Vehicle1 truck = ( *this );
  Vehicle1 other = otherTruck;

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
  int iHighLimit = std::min( truck.size(), fromPos + 5 );
  int jLowLimit = std::max( POS(1), toPos - 5 );
  int jHighLimit = std::min( other.size(), toPos + 5 );

  for ( int i = iLowLimit; i < iHighLimit; i++ ) {
    if (i == 0) continue;
    if (!(i < truck.size())) continue;;

    if ( truck.path[i].isDump() ) continue;

    fromNodeId = truck.path[i].nid();

    for ( int j = jLowLimit; j < jHighLimit - 1; j++ ) {
      if (j == 0) continue;
      if (!(j < other.size())) continue;
      if ( other.path[j].isDump() ) continue;

      otherDelta = other.timePCN(j - 1, truck.path[i]) 
                    - other.timePCN(j - 1, j, j + 1);
      truckDelta = truck.timePCN(i - 1, other.path[j]) 
                    - truck.timePCN(i - 1, i, i + 1);

      toNodeId = other.path[j].nid();

      if ( otherDelta < 0 or truckDelta < 0 ) {
        savings = VRP_MIN();

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

long int Vehicle1::eval_interSwapMoveDumps( Moves &moves,
    const Vehicle1 &otherTruck, POS  truckPos, POS  otherTruckPos,
    double factor ) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::eval_interSwapMoveDumps" );
#endif
#ifdef TESTED
  DLOG( INFO ) << "Entering Vehicle1::eval_interSwapMoveDumps";
#endif
  //double minSavings = -5;

  Vehicle1 truck = ( *this );
  Vehicle1 other = otherTruck;
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


          savings = VRP_MIN();

          if ( otherDelta < 0 or truckDelta < 0 ) {
            if ( truck.applyMoveInterSw( other, i, j ) ) {
              newCost = truck.getCost() + other.getCost();
              //newDuration = truck.getDuration() + other.getDuration();
              savings = originalCost - newCost;
              move.setInterSwMove( truckPos,  i,  fromNodeId,  otherTruckPos, j, toNodeId,
                                   savings );
              moves.insert( move );
              deltaMovesSize++;
            } else numNotFeasable++;

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
bool Vehicle1::e_insertIntoFeasableTruck( const Trashnode &node, POS pos )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::e_insertIntoFeasableTruck" );
#endif
#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering Vehicle1::e_insertIntoFeasableTruck";
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
bool Vehicle1::eval_erase(int at, double &savings) const {
#ifdef DOSTATS
 STATS->inc("Vehicle1::eval_erase");
#endif
#ifdef TESTED
DLOG(INFO) << "Entering Vehicle1::eval_erase";
#endif
    assert (at<size() and at>0 );
    if ( path[at].isdump() ) { savings=_MIN(); return false;}
    Vehicle1 truck = (*this);
    truck.path.erase(at);
    if ( not truck.e_makeFeasable(at) ) savings = _MIN(); // -infinity
        else savings = cost - truck.cost;

    return truck.feasable();
};
*/


//dont forget, negative savings is a higher cost
bool Vehicle1::eval_erase( POS at, double &savings ) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::eval_erase" );
#endif
#ifdef TESTED
  DLOG( INFO ) << "Entering Vehicle1::eval_erase";
#endif
  assert ( at<size() and at>0 );

  if ( path[at].isDump() ) { savings = VRP_MIN(); return false;}

  Vehicle1 truck = ( *this );
  double oldcost = truck.getCost();

  truck.path.erase( at );

  if ( not truck.e_makeFeasable( at ) ) savings = VRP_MIN(); // -infinity
  else savings = oldcost - truck.getCost();

#ifdef TESTED
  DLOG( INFO ) << "ERASE: oldcost: " << oldcost
               << "\tnewcost: " << truck.getCost()
               << "\tsavings: " << oldcost - truck.getCost();
#endif
  return truck.feasable();
};

long int Vehicle1::eval_insertMoveDumps( const Trashnode &node, Moves &moves,
                                        POS fromTruck, POS fromPos, POS toTruck,
                                        double eraseSavings) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::eval_insertMoveDumps" );
#endif
#ifdef TESTED
  DLOG( INFO ) << "Entering Vehicle1::eval_insertMoveDumps";
#endif
  Vehicle1 truck = ( *this );
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
    truck.e_insert( node, currentPos );

    if ( not truck.e_makeFeasable( currentPos ) ) {
#ifdef TESTED
      truck.tau();
      truck.dumpeval();
#endif
      impossiblePos.push_back( currentPos );

      //if ( path.size()*factor > impossiblePos.size() ) return moves.size();
    } else {
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



bool Vehicle1::e_makeFeasable( POS currentPos )
{
#ifdef DOSTATS
  STATS->inc( " Vehicle1::e_makeFeasable" );
#endif
#ifdef TESTED
  DLOG( INFO ) << "Entering Vehicle1::e_makeFeasable";
#endif
  path.e__adjustDumpsToMaxCapacity( currentPos, dumpSite, maxcapacity );
  evalLast();
  return feasable();
}

bool Vehicle1::applyMoveINSerasePart( UID nodeNid, POS pos )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::applyMoveINSerasePart" );
#endif
#ifdef TESTED
  DLOG( INFO ) << "Entering Vehicle1::applyMoveINSerasePart";
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


bool Vehicle1::applyMoveINSinsertPart( const Trashnode &node, POS pos )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::applyMoveINSinsertPart" );
#endif
  path.insert( node, pos );
  e_makeFeasable( pos );

#ifdef DOVRPLOG
  if ( not feasable() ) dumpeval();
#endif

  assert ( feasable() );
  return feasable();
}

bool Vehicle1::applyMoveInterSw( Vehicle1 &otherTruck, POS truckPos,
                                POS otherTruckPos )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::applyMoveInterSw" );
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

bool Vehicle1::applyMoveIntraSw( POS  fromPos, POS withPos )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::applyMoveIntraSw" );
#endif
  path.swap( fromPos,  withPos );

  if ( not e_makeFeasable( std::min( fromPos - 1, withPos ) ) ) return false;

  //evalLast(); done in makeFeasable
  assert ( feasable() );
  return feasable() ;
}


#if 0
//NOT USED
bool Vehicle1::e_insertMoveDumps( const Trashnode &node, int at)
{
  assert (at <= size());
  path.insert(node, at);
  path.e_moveDumps(at);
}

// Very TIGHT insertion
// insertion will not be performed if
//      TV and CV are  generated
//  true- insertion was done
//  false- not inserted
bool Vehicle1::e_insertSteadyDumpsTight( const Trashnode &node, POS at )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::e_insertSteadyDumpsTight" );
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


bool Vehicle1::e_insertDumpInPath( const Trashnode &lonelyNodeAfterDump )
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::e_insertDumpInPath" );
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







//bool Vehicle1::deltaCargoGeneratesCV_AUTO(const Trashnode &node, int pos) const { //position becomes important

bool Vehicle1::deltaCargoGeneratesCV( const Trashnode &node,
                                     POS pos ) const   //position becomes important
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::deltaCargoGeneratesCV" );
#endif
#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering Vehicle1::deltaCargoGeneratesCV";
  DLOG(INFO) << getCargo() << "+" << node.getDemand() << " ¿? " <<
             getmaxcapacity();
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
bool Vehicle1::deltaTimeGeneratesTV( const Trashnode &dump,
                                    const Trashnode &node ) const
{
#ifdef DOSTATS
  STATS->inc( " Vehicle1::deltaTimeGeneratesTV" );
#endif

#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering Vehicle1::deltaTimeGeneratesTV";
  DLOG( INFO ) << " (S 1 2 3 D E )  (S 1 2 3 D N D E)"
               << path.getDeltaTimeAfterDump( dump, node ) << " + "
               << getDuration() << " ¿? " <<  endingSite.closes();
#endif

  return  ( path.getDeltaTimeAfterDump( dump, node ) +
            getDuration()  > endingSite.closes() );
}





bool Vehicle1::deltaTimeGeneratesTV( const Trashnode &node, POS pos ) const
{
#ifdef DOSTATS
  STATS->inc( "Vehicle1::deltaTimeGeneratesTV" );
#endif
  assert( pos <= path.size() );

#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering Vehicle1::deltaTimeGeneratesTV";

  if ( pos > path.size() )
    DLOG( INFO ) << "CANT work with this pos:" << pos;

  if ( pos == path.size() )
    DLOG(INFO) << " ( S 1 2 3 D E )  ( S 1 2 3 N D E )"
               << path.getDeltaTime(node, dumpSite) << " + "
               << getDuration() << " ¿ ? " <<  endingSite.closes();
  else
    DLOG(INFO) << " ( S 1 2 3 D E )  ( S 1 2 N 3 D E ) "
               << path.getDeltaTime(node, pos) << " + "
               << getDuration() << " ¿ ? " <<  endingSite.closes();

  endingSite.dump();
#endif

  if ( pos == path.size() )
    return path.getDeltaTime( node, dumpSite ) +
           getDuration() > endingSite.closes();
  else
    return path.getDeltaTime( node, pos ) +
           getDuration() > endingSite.closes();
}
#endif

