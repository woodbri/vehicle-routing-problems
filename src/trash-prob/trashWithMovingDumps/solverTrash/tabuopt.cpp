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
#include <deque>
#include <algorithm>
#include <cstdlib>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "stats.h"
#include "timer.h"
#endif


#include "optsol.h"
#include "tabuopt.h"



TabuOpt::TabuOpt( const OptSol &initialSolution, unsigned int iteration ) :
  TabuBase( initialSolution )
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::TabuOpt" );
  Timer start;
#endif
  maxIteration = iteration;

  if (maxIteration == 0) return;

  bestSolution.evaluate();
  computeCosts( bestSolution );
#ifdef VRPMINTRACE
  bestSolution.tau();
#endif
  bestSolution.optimizeTruckNumber();
  bestTabuList.clear();
  bestSolution.evaluate();
  computeCosts(bestSolution);
  bestSolutionCost = bestSolution.getCost();
  setBestAsCurrent();
#ifdef VRPMINTRACE
#ifdef DOSTATS
  DLOG( INFO ) << "TABUSEARCH: Removal of truck time: " << start.duration();
#endif
  bestSolution.evaluate();
  bestSolution.dumpCostValues();
  bestSolution.tau();
#endif

  limitIntraSw = bestSolution.getFleetSize();
  limitInterSw = limitIntraSw * ( limitIntraSw - 1 ) / 2 ;
  limitIns    = limitInterSw;
#ifdef DOSTATS
  STATS->set( "limitIntraSw", limitIntraSw );
  STATS->set( "limitInterSw", limitIntraSw );
  STATS->set( "limitIns", limitInterSw );
#endif
  currentIteration = 0;

  if (maxIteration > 1) search();

  bestSolution.evaluate();
};


vehicle_path_t *TabuOpt::getSolutionForPg( UINT &count ) const
{
  return bestSolution.getSolutionForPg(count);
}





/**
    This Tabu search algorithm was adapted from the paper:

    "Tabu Search Techniques for the Hetrogeneous Vehicle Routing
    Problem with Time Windows and Carrier-Dependent Cots" by
    Sara Ceschia, Luca Di Gaspero, and Andrea Schaerf

    We use a sequential solving strategy for combining our three neighborhood
    move functions. This as know as a "token-ring" search. Given an initial
    state and a set of algorithms, it makes circularly a run at each algorithm,
    always starting from the best solution found by the previous one. The
    overall process stops either when a full round of the algorithms does not
    find an improvement or the time (aka: interation count) granted has elapsed.

    Each single algorithm stops when it does not improve the current best
    solution for a given number of iterations (ie: stagnation).

*/
void TabuOpt::search()
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::search" );
  Timer start;
#endif


  bool improvedBest;
  //int lastImproved = 0;
  double oldCost = currentSolution.getCost();
  double newCost;
  int cycleLimit = 5;

  //first cycle with osrm
#ifdef OSRMCLIENT
  osrmi->useOsrm( true );
#endif

  for ( int i = 0; i < maxIteration; i++ ) {
#ifdef  DOVRPLOG
    DLOG( INFO ) << "TABUSEARCH: Starting iteration: " << currentIteration;
#endif

    THROW_ON_SIGINT

#if defined (OSRMCLIENT) && defined (VRPMINTRACE)

    if ( osrmi->getUse() )
      DLOG( INFO ) << "OSRM set to be used";
    else DLOG( INFO ) << "OSRM set to be not used";

#endif

    oldCost = currentSolution.getCost();

#ifdef DOSTATS
    start.restart();
#endif


#ifdef OSRMCLIENT

    if ( osrmi->getUse() ) cycleLimit = 1;
    else cycleLimit = 5;

#endif

    improvedBest = doNeighborhoodMoves( Move::IntraSw, 1  );

    for ( int j = 0; j < cycleLimit; j++ ) {
#ifdef VRPMINTRACE
      DLOG( INFO ) << "----------------- TABUSEARCH: InterSw: " << j;
#endif
      improvedBest |= doNeighborhoodMoves( Move::InterSw, 1  );
    }

    THROW_ON_SIGINT

#ifdef VRPMINTRACE
    currentSolution.tau();
#endif

#ifdef VRPMAXTRACE
    for ( int j = 0; j < cycleLimit; j++ ) {
#ifdef VRPMINTRACE
      DLOG( INFO ) << "------------------TABUSEARCH: Ins: " << j;
#endif
      improvedBest |= doNeighborhoodMoves( Move::Ins, 1 );
    }
#endif

    THROW_ON_SIGINT

#ifdef VRPMINTRACE
    currentSolution.tau();
#endif

    //TODO I would like to make the tabu moves here

    newCost = currentSolution.getCost();


#ifdef DOVRPLOG
    DLOG( INFO ) << "TABUSEARCH: Finished iteration: " << currentIteration
                 << "\nold cost" << oldCost
                 << "\nnew cost" << newCost
                 << "\nbest cost" << bestSolution.getCost()
                 << "\ndifference" << std::abs( newCost - oldCost ) << "\n";
#endif

#ifdef DOSTATS
    DLOG( INFO ) << ", run time in seconds: " << start.duration();
    STATS->set( "Iteration", currentIteration );
    STATS->set( "Best Cost After", bestSolution.getCost() );
#endif

#ifdef VRPMINTRACE
    dumpStats();
    DLOG( INFO ) << "--------------------------------------------";
#endif

    currentIteration++;

#ifndef OSRMCLIENT

    if ( std::abs( newCost - oldCost ) < 0.1  ) {
#ifdef VRPMINTRACE
      DLOG( INFO ) << "costs didnt change";
#endif
      break;
    }

#else


    if ( std::abs( newCost - oldCost ) > 0.5 )   osrmi->useOsrm ( false );
    else {
      if ( osrmi->getUse() == true ) {
#ifdef VRPMINTRACE
        DLOG( INFO ) << "costs didnt change quiting";
#endif
        break;
      } else {
#ifdef VRPMINTRACE
        DLOG( INFO ) << "costs didnt change TRYING with OSRM";
#endif
        osrmi->useOsrm ( true );
        continue;
      }
    }

#endif
  }

#ifdef DOSTATS
  DLOG( INFO )  << "TABUSEARCH: Total time: " << start.duration();
#endif
#ifdef DOVRPLOG
  bestSolution.tau();
#endif
}


void TabuOpt::getNeighborhood(  Move::Mtype  whichNeighborhood,
                                Moves &neighborhood, double factor ) const
{
  assert (whichNeighborhood != Move::Invalid);
  neighborhood.clear();
#ifdef DOSTATS
  STATS->inc( "TabuOpt::getNeighborhood" );
  Timer getNeighborhoodTimer;
#endif

  switch ( whichNeighborhood ) {
  case Move::Ins:
    currentSolution.getInsNeighborhood( neighborhood );
#ifdef DOSTATS
    generateNeighborhoodStats( "Ins", getNeighborhoodTimer.duration(),
                               neighborhood.size() );
#endif
    break;

  case Move::IntraSw:
    currentSolution.getIntraSwNeighborhood( neighborhood );
#ifdef DOSTATS
    generateNeighborhoodStats( "IntraSw", getNeighborhoodTimer.duration(),
                               neighborhood.size() );
#endif
    break;

  case Move::InterSw:
    currentSolution.getInterSwNeighborhood( neighborhood, factor );
#ifdef DOSTATS
    generateNeighborhoodStats( "InterSw", getNeighborhoodTimer.duration(),
                               neighborhood.size() );
#endif
    break;

  case Move::Invalid:
    break;
  }
}


bool TabuOpt::applyAmove( const Move &move )
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::applyAmove" );
#endif
#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Apply a move ";
  move.Dump();
#endif

  currentSolution.v_applyMove( move );
  makeTabu( move );
  computeCosts( currentSolution );
  currentCost = currentSolution.getCost();

  if ( bestSolutionCost > currentCost ) {
    setCurrentAsBest();
#ifdef DOSTATS
    STATS->set( "best Updated Last At Iteration", currentIteration );
    STATS->inc( "number of times best was Updated " );
#endif
  }

  return true;
}


bool TabuOpt::applyMoves(const std::string &type, Moves &moves )
{
  if ( not moves.size() ) return false;

  bool retValue;
  Move move =  *(moves.begin());
  retValue = applyAmove( move );
  cleanUpMoves( move );

#ifdef VRPMINTRACE
  DLOG( INFO ) << "apply moves: " << type << " applied: ";
  move.Dump();
#endif
#ifdef DOSTATS
  STATS->inc( "Number of moves of type " + type );
#endif

  return retValue;
}


bool TabuOpt::reachedMaxCycles( int number,  Move::Mtype whichNeighborhood )
{
  assert (whichNeighborhood != Move::Invalid);
#ifdef DOSTATS
  STATS->inc( "TabuOpt::reachedMaxCycles" );
#endif
#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering TabuOpt::reachedMaxCycles " << number;
#endif
  bool limit;

  switch ( whichNeighborhood ) {
  case Move::Ins: { limit = number >= limitIns; break;}

  case Move::IntraSw: { limit = number >= limitIntraSw; break;}

  case Move::InterSw: { limit = number >= limitInterSw; break;}

  case Move::Invalid: { limit = true; break;}
  };

  return limit;
}

/*
    Algorithm for processing neighborhood moves [article]

    For the requested individual move neighborhood:
        doInsMoves, doIntraSwMoves, doInterSwMoves

    do {
        Generate the neighborhood of moves and order from best to worst.
        Working through the neighborhood (best to worst)
        Filter out inFeasible moves then
        if the move is aspirational we apply the move
        otherwise if the move is not Tabu apply the move
        even if it makes the current solution worse.
        If all moves are tabu, then apply the best one
    } until there are not valid moves or stagnation
    return an indicator that we improved the best move or not.
*/

bool TabuOpt::doNeighborhoodMoves( Move::Mtype whichNeighborhood, int maxMoves)
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::doNeighborhoodMoves" );
#endif

#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering TabuOpt::doNeighobrhoodMoves";
#endif
  bool improvedBest = false;
  int Cnt = 0;
  //int CntNonAspirational = 0;
  int CntNoNeighborhood = 0;
  double factor = 0.02;
  //bool limit;
  int actualMoveCount = getTotalMovesMade();
  bool moveMade = false;
  bool intraSwMoveMade = true;


  Moves neighborhood;

  do {

    THROW_ON_SIGINT

#ifdef VRPMINTRACE
    DLOG( INFO ) << ( getTotalMovesMade() - actualMoveCount ) << " > " << maxMoves
                 << " ***************************************************";
    DLOG( INFO ) << " Factor  " << factor;
#endif

    if (factor > 1 ) break;
    if ( ( getTotalMovesMade() - actualMoveCount ) > maxMoves ) break;

    //if (whichNeighborhood==Move::Ins) factor=1;
    if ( factor == 0.01 and CntNoNeighborhood == 0 and notTabu.size() == 0
         and tabu.size() == 0 )
      getNeighborhood( whichNeighborhood, neighborhood, 1 );
    else
      getNeighborhood( whichNeighborhood, neighborhood, factor );

    Cnt++;

    if ( whichNeighborhood == Move::IntraSw ) {
      //do as many as the bookkeeping allows
      while ( neighborhood.size() ) {
        applyMoves( "IntraSw", neighborhood );
        neighborhood.erase( neighborhood.begin() );
      }

      return true;
    }


    if ( not neighborhood.size() ) {
      //need to increase the search space
      factor = std::min( factor + 1.0 / limitInterSw, factor + 0.3 );

      CntNoNeighborhood++;
#ifdef DOVRPLOG
      DLOG( INFO ) << " No Moves are found  " << CntNoNeighborhood;
      DLOG( INFO ) << " Factor  " << factor;
#endif

      if ( reachedMaxCycles( CntNoNeighborhood, whichNeighborhood ) or
           whichNeighborhood == Move::IntraSw ) {
#ifdef DOVRPLOG
        DLOG( INFO ) << " Reached end of cycle - for No moves found- " << Cnt
                     << " out of " << maxMoves;
#endif
        return improvedBest; //we cycled and no neighborhood moves were found
      };

      //if (not notTabu.size() and not tabu.size() ) break;
    } else  CntNoNeighborhood = 0;

    std::string solAfter  = currentSolution.solutionAsText();


    if ( classifyMoves( neighborhood ) ) {
      assert ( not neighborhood.size() );
      assert ( not aspirationalTabu.size() );
      assert ( not notTabu.size() );
      assert ( not tabu.size() );
      improvedBest = true;
      moveMade = true;
      Cnt = 0; //a move was made and it reduced the number of trucks
      continue;
    }

    if ( aspirationalNotTabu.size() ) {
      improvedBest = true;
      moveMade = true;
      factor = 0.01; //trully tryllu optimistic

      while ( aspirationalNotTabu.size() ) { //do as many as the bookkeeping allows
        applyMoves( "aspirational non tabu", aspirationalNotTabu );
        Cnt = 0;
      }
    }



    if ( aspirationalTabu.size() ) {
      improvedBest = true;
      moveMade = true;
      factor = 0.01; //trully tryllu optimistic

      while ( aspirationalTabu.size() ) { //do as many as the bookkeping allows
        applyMoves( "aspirational Tabu",  aspirationalTabu );
        Cnt = 0;
      }
    }

    factor = std::min( factor + 1.0 / limitInterSw,
                       factor + 0.3 ); //need to increase the search space

    if ( notTabu.size() and  notTabu.begin()->getsavings() > 0 )   {
      improvedBest = true;
      moveMade = true;

      while ( notTabu.size() ) {
        applyMoves( "not Tabu with pos savings",  notTabu );

        if ( notTabu.begin()->getsavings() < 0 )
          notTabu.clear(); // only apply positives

        Cnt = 0;
      }
    }

    if ( moveMade == true ) break;

    if ( ( not ( whichNeighborhood == Move::IntraSw ) and intraSwMoveMade ) ) {
      improvedBest =  doNeighborhoodMoves( Move::IntraSw, 1 );

      if ( improvedBest ) {
        actualMoveCount = getTotalMovesMade();
        intraSwMoveMade = true;
      } else intraSwMoveMade = false;
    }

    //if ( (whichNeighborhood==Move::Ins and (currentSolution.getFleetSize()==2) )
    if (   ( notTabu.size() and factor > 0.9 and not intraSwMoveMade
             and reachedMaxCycles( Cnt, whichNeighborhood ) ) )  {
      while ( notTabu.size() ) {
        applyMoves( "not Tabu",  notTabu );
        Cnt = 0;
      }

      moveMade = true;
      continue;
    }

    if ( ( whichNeighborhood == Move::Ins
           and ( currentSolution.getFleetSize() == 2 ) )
         or  ( factor > 0.9  and  reachedMaxCycles( Cnt, whichNeighborhood ) ) ) {
      while ( tabu.size() ) {
        applyMoves( "tabu", tabu );
        moveMade = true;
      }
    }
  } while ( not moveMade );

  if ( not intraSwMoveMade )
    improvedBest |= doNeighborhoodMoves( Move::IntraSw, 1 );

  return improvedBest;
}


/**
  Classify the moves into:
    aspirational Not tabu   (if found the buckets bellow are cleared)
    aspirational tabu   (adds to the bucket the best move)
    not Tabu        (adds to the bucket the best move)
    Tabu            (adds to the bucket all the moves)

  if during the classification a truck is removed:
    move is applied
    all buckets are cleared


    returns true: the truck number was reduced
*/
bool TabuOpt::classifyMoves( Moves &neighborhood )
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::classifyMoves" );
  Timer start;
#endif

  //bool found = false;
  //bool foundAspTabu = false;
  bool removedTruck = false;
  double newCost;
  computeCosts( currentSolution );
#ifdef VRPMAXTRACE
  double actualCost = currentSolution.getCost();
#endif
  OptSol current = currentSolution;
  Move guide;

  for ( MovesItr it = neighborhood.begin();
        it != neighborhood.end(); ++it ) {
    current = currentSolution;
    current.v_applyMove( *it );
    removedTruck = computeCosts( current );
    newCost = current.getCost();

    if ( removedTruck ) {
      currentSolution = current;
      setCurrentAsBest();
      makeTabu( *it );
#ifdef DOSTATS
      STATS->set( "best Updated Last At", currentIteration );
      STATS->inc( "best Updated Cnt" );
#endif
      //clear up all buckets
      neighborhood.clear();
      aspirationalNotTabu.clear(); aspirationalTabu.clear(); notTabu.clear();
      tabu.clear();
      return true;
    }

#ifdef VRPMAXTRACE
    DLOG( INFO ) << "isTabu??: " << ( isTabu( *it ) ? "YES" : "NO" );
    it->Dump();

    if ( not ( ( std::abs( ( actualCost - newCost )  -  it->getsavings() ) ) <
               0.5 ) ) {
      it->Dump();
      DLOG( INFO ) << "something is wrong with the savings****** "
                   << it->getsavings() << " ***** " << actualCost - newCost;
    }

#endif

    if ( newCost  < bestSolutionCost and not isTabu( *it ) )
      aspirationalNotTabu.insert( *it );
    else if ( newCost  < bestSolutionCost )
      aspirationalTabu.insert( *it );
    else if ( not isTabu( *it ) )
      notTabu.insert( *it );
    else
      tabu.insert( *it );

  };

  neighborhood.clear();
  return false;         //we didnt make a move that deleted a truck
};

bool TabuOpt::computeCosts( OptSol &s )
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::computeCosts" );
#endif
  int removedTruck = s.v_computeCosts();

  if ( removedTruck == -1 ) return false;

  removeTruckFromTabuList( removedTruck ) ;
  return true;
}


void TabuOpt::cleanUpInterSwMoves( Moves &moves, const Move &guide ) const
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::cleanUpInterSwMoves" );
#endif

  if ( not moves.size() ) return;

  if ( not guide.getmtype() == Move::InterSw ) return;

  int fromPos = guide.getInterSwFromPos();
  int toPos = guide.getInterSwToPos();
  Moves oldMoves = moves;
  moves.clear();
  Move move;

  for ( MovesItr movePtr = oldMoves.begin(); movePtr != oldMoves.end();
        ++movePtr ) {
    move = ( *movePtr );
    //nothing changes position when an interswp move is done, except for the containers
    //that where changed, and probably at the end of the route because of the moving dumps

    if (    ( move.getvid1() != guide.getInterSwTruck1() )
            and ( move.getvid1() != guide.getInterSwTruck2() )
            and ( move.getvid2() != guide.getInterSwTruck1() )
            and ( move.getvid2() != guide.getInterSwTruck2() ) ) {
      //moves that dont have the trucks involved in the interSw move are ok
      assert ( currentSolution.testInterSwMove( move ) );

      if ( not currentSolution.testInterSwMove( move ) ) continue;

      moves.insert( move );
    }

    //only moves that invlove the trucks of the intraswp move are analyzed



    //moves with nodes involved on move done are discarded
    if ( move.getnid1() == guide.getnid1() ) continue;

    if ( move.getnid1() == guide.getnid2() ) continue;

    if ( move.getnid2() == guide.getnid1() ) continue;

    if ( move.getnid2() == guide.getnid2() ) continue;

    //avoid changes because of moving dumps
    //all moves involving the "end" of the path are discarded
    if ( currentSolution[move.getvid1()].size() - 5 <= move.getpos1() ) continue;

    if ( currentSolution[move.getvid2()].size() - 5 <= move.getpos2() ) continue;

    switch ( move.getmtype() ) {
    case Move::InterSw: {
        //get rid of neighboring moves, savigs have being changed
        if ( move.getInterSwTruck1() == guide.getInterSwTruck1() ) {
          if ( inRange( fromPos, move.getInterSwFromPos(), 2 ) ) continue;
        }

        if ( move.getInterSwTruck1() == guide.getInterSwTruck2() ) {
          if ( inRange( toPos, move.getInterSwFromPos(), 2 ) ) continue;
        }

        if ( move.getInterSwTruck2() == guide.getInterSwTruck1() ) {
          if ( inRange( fromPos, move.getInterSwToPos(), 2 ) ) continue;
        }

        if ( move.getInterSwTruck2() == guide.getInterSwTruck2() ) {
          if ( inRange( toPos, move.getInterSwToPos(), 2 ) ) continue;
        }

        //moves that survived maybe are not feasable now becuase of moving dumps
        if ( not currentSolution.testInterSwMove( move ) ) continue;

        moves.insert( move );
        break;
      }

    case Move::Ins: {
        //get rid of neighboring moves, savigs have being changed
        if ( move.getInsFromTruck() == guide.getInterSwTruck1() ) {
          if ( inRange( fromPos, move.getInsFromPos(), 2 ) ) continue;
        }

        if ( move.getInsFromTruck() == guide.getInterSwTruck2() ) {
          if ( inRange( toPos, move.getInsFromPos(), 2 ) ) continue;
        }

        if ( move.getInsToTruck() == guide.getInterSwTruck1() ) {
          if ( inRange( fromPos, move.getInsToPos(), 2 ) ) continue;
        }

        if ( move.getInsToTruck() == guide.getInterSwTruck2() ) {
          if ( inRange( toPos, move.getInsToPos(), 2 ) ) continue;
        }

        //moves that survived maybe are not feasable now becuase of moving dumps
        if ( not currentSolution.testInterSwMove( move ) ) continue;

        moves.insert( move );
        break;
      }

    case ( Move::IntraSw ): {
        //in theory no intraSw is here, so skip them
        continue;
      }

    }
  }
}


void TabuOpt::cleanUpInsMoves( Moves &moves, const Move &guide,
                               bool &reverseFound )
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::cleanUpInsMoves" );

#endif

  if ( not moves.size() ) return;

  //int fromPos = guide.getInsFromPos();
  //POS toPos = guide.getInsToPos();
  Moves oldMoves = moves;
  moves.clear();

  if ( not guide.getmtype() == Move::Ins ) return;

  Move move;
  Move reverseMove;
  bool localFind = false;

  for ( MovesItr movePtr = oldMoves.begin(); movePtr != oldMoves.end();
        ++movePtr ) {
    move = ( *movePtr );

    if ( not ( move.getmtype() == Move::Ins ) ) continue;

    if ( move == guide ) continue;

    if ( not reverseFound and not localFind
         and move.getInsFromTruck() == guide.getInsToTruck()
         and move.getInsToTruck() == guide.getInsFromTruck() ) { //making a reverse move

      if ( move.getInsFromPos() == guide.getInsToPos()
           or  ( move.getInsFromPos() == guide.getInsToPos() + 1 )
           or  ( move.getInsFromPos() == guide.getInsToPos() - 1 )
           or  ( move.getInsFromPos() == guide.getInsToPos() + 2 )
           or  ( move.getInsFromPos() == guide.getInsToPos() - 2 ) )
        continue;

      if ( move.getInsFromPos() > guide.getInsToPos() ) move.setInsFromPos(
          move.getInsFromPos() + 1 );

      if ( move.getInsToPos() > guide.getInsFromPos() ) move.setInsToPos(
          move.getInsToPos() - 1 );

      if ( not currentSolution.testInsMove( move ) ) continue;

      reverseMove = move;
      localFind = true;
      continue;
    }

    if ( move.getInsFromTruck() == guide.getInsFromTruck()
         or move.getInsToTruck() == guide.getInsFromTruck() ) continue;

    if ( move.getInsFromTruck() == guide.getInsToTruck()
         or move.getInsToTruck() == guide.getInsToTruck() ) continue;

    moves.insert( move );
  }

  if ( localFind ) {
#ifdef VRPMAXTRACE
    DLOG( INFO ) << "REVERSE going to make";
#endif
    applyAmove( reverseMove );
#ifdef VRPMAXTRACE
    DLOG( INFO ) << "REVERSE done";
#endif
    reverseFound = true;
  }
}



bool TabuOpt::inRange( int center, int data, int step ) const
{
  return ( ( ( center - step ) <= data ) and ( data <= ( center + step ) ) ) ;
}

void TabuOpt::cleanUpIntraSwMoves( Moves &moves, const Move &guide ) const
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::cleanUpIntraSwMoves " );
#endif

  if ( not moves.size() ) return;

  Moves oldMoves = moves;
  moves.clear();
  Move move;

  if ( not ( guide.getmtype() == Move::IntraSw ) ) return;

  POS truckPos = guide.getIntraSwTruck();

  for ( MovesItr movePtr = oldMoves.begin(); movePtr != oldMoves.end();
        ++movePtr ) {
    move = ( *movePtr );

    switch ( move.getmtype() ) {
    case Move::InterSw: {

        if ( move.getInterSwTruck1() == truckPos ) {
          if ( inRange( guide.getIntraSwFromPos(), move.getInterSwFromPos(),
                        2 ) ) continue;

          if ( inRange( guide.getIntraSwToPos(), move.getInterSwFromPos(), 2 ) ) continue;

          if ( currentSolution[truckPos].size() - 5 <= move.getInterSwFromPos() )
            continue;

          if ( not currentSolution.testInterSwMove( move ) ) continue;

          moves.insert( move );
        }

        if ( move.getInterSwTruck2() == truckPos ) {
          //The sorrounding Positions structure of the truck are not the same
          if ( inRange( guide.getIntraSwFromPos(), move.getInterSwToPos(), 2 ) ) continue;

          if ( inRange( guide.getIntraSwToPos(), move.getInterSwToPos(), 2 ) ) continue;

          if ( currentSolution[truckPos].size() - 5 <= move.getInterSwToPos() ) continue;

          if ( not currentSolution.testInterSwMove( move ) ) continue;

          moves.insert( move );
        }

        break;
      }

    case Move::Ins: {

        if ( move.getInsFromTruck() == truckPos ) {
          //The sorrounding Positions structure of the truck are not the same
          if ( inRange( guide.getIntraSwFromPos(), move.getInsFromPos(), 2 ) ) continue;

          if ( inRange( guide.getIntraSwToPos(), move.getInsFromPos(), 2 ) ) continue;

          if ( currentSolution[truckPos].size() - 3 <= move.getInsFromPos() ) continue;

          moves.insert( move );
        }

        if ( move.getInsToTruck() == truckPos ) {
          //The sorrounding Positions structure of the truck are not the same
          if ( inRange( guide.getIntraSwFromPos(), move.getInsToPos(), 2 ) ) continue;

          if ( inRange( guide.getIntraSwToPos(), move.getInsToPos(), 2 ) ) continue;

          if ( currentSolution[truckPos].size() - 3 <= move.getInsToPos() ) continue;

          moves.insert( move );
        }

        break;
      }

    case ( Move::IntraSw ): {

        if ( truckPos == move.getIntraSwTruck() ) continue;

        if ( move.getIntraSwNid1() == guide.getIntraSwNid1() ) continue;

        if ( move.getIntraSwNid2() == guide.getIntraSwNid2() ) continue;

        moves.insert( move );
      }
    }
  }
}




void TabuOpt::cleanUpMoves( const Move guide )
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::cleanUpMoves" );
#endif
#ifdef VRPMAXTRACE
  DLOG( INFO ) << "ENTERING TabuOpt::cleanUpMoves";

  if ( guide.getmtype() == Move::Ins ) {
    guide.Dump();

    if ( aspirationalNotTabu.size() ) DLOG( INFO ) <<
          "cleaning aspirational not tabu";

    if ( aspirationalTabu.size() ) DLOG( INFO ) << "cleaning aspirational tabu";

    if ( notTabu.size() ) DLOG( INFO ) << "cleaning not tabu";

    if ( tabu.size() ) DLOG( INFO ) << "cleaning tabu";

    DLOG( INFO ) << "aspirational not Tabu size" << aspirationalNotTabu.size();
    dumpMoves( "aspirationalNotTabu", aspirationalNotTabu );
    DLOG( INFO ) << "aspirationalTabu size" << aspirationalTabu.size();
    dumpMoves( "aspirationalTabu", aspirationalTabu );
    DLOG( INFO ) << "not Tabu size" << notTabu.size();
    dumpMoves( "notTabu", notTabu );
    DLOG( INFO ) << "Tabu size" << tabu.size();
    dumpMoves( "tabu", tabu );
  }

#endif

  switch ( guide.getmtype() ) {
  case Move::InterSw:
    if ( aspirationalNotTabu.size() ) cleanUpInterSwMoves( aspirationalNotTabu,
          guide );

    if ( aspirationalTabu.size() ) cleanUpInterSwMoves( aspirationalTabu, guide );

    if ( notTabu.size() ) cleanUpInterSwMoves( notTabu, guide );

    if ( tabu.size() ) cleanUpInterSwMoves( tabu, guide );

    break;

  case Move::IntraSw:
    if ( aspirationalNotTabu.size() ) cleanUpIntraSwMoves( aspirationalNotTabu,
          guide );

    if ( aspirationalTabu.size() ) cleanUpIntraSwMoves( aspirationalTabu, guide );

    if ( notTabu.size() ) cleanUpIntraSwMoves( notTabu, guide );

    if ( tabu.size() ) cleanUpIntraSwMoves( tabu, guide );

    break;

  case Move::Ins:
    bool reverseFound = false;

    if ( aspirationalNotTabu.size() ) cleanUpInsMoves( aspirationalNotTabu, guide,
          reverseFound );

    if ( aspirationalTabu.size() ) cleanUpInsMoves( aspirationalTabu, guide,
          reverseFound );

    if ( notTabu.size() )cleanUpInsMoves( notTabu, guide, reverseFound );

    if ( tabu.size() ) cleanUpInsMoves( tabu, guide, reverseFound );

    break;
  }

#ifdef VRPMAXTRACE
  DLOG( INFO ) << "aspirational not Tabu size" << aspirationalNotTabu.size();
  dumpMoves( "aspirationalNotTabu", aspirationalNotTabu );
  DLOG( INFO ) << "aspirationalTabu size" << aspirationalTabu.size();
  dumpMoves( "aspirationalTabu", aspirationalTabu );
  DLOG( INFO ) << "not Tabu size" << notTabu.size();
  dumpMoves( "notTabu", notTabu );
  DLOG( INFO ) << "Tabu size" << tabu.size();
  dumpMoves( "tabu", tabu );
#endif
}




void TabuOpt::dumpMoves( std::string str, Moves moves ) const
{
#ifdef DOSTATS
  STATS->inc( "TabuOpt::dumpMoves" );
#endif
#ifdef DOVRPLOG
  DLOG( INFO ) << "Bucket: " << str;
#endif

  MovesItr movePtr;

  for ( movePtr = moves.begin(); movePtr != moves.end(); ++movePtr )
    movePtr->Dump();
};

