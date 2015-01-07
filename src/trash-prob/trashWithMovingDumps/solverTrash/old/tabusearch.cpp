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

#include "stats.h"
#include "neighborhoods.h"
#include "tabusearch.h"


/*
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

void TabuSearch::search()
{

  currentIteration = 0;

  Timer start;
  bool improvedBest;
  int lastImproved = 0;

  do {
    std::cout << "TABUSEARCH: Starting iteration: " << currentIteration
              << std::endl;
    // [article]
    // set the stagnation count as the last the last parameter
    // the values 500, 300, 300 can from the paper mentioned above
    //
    // this is a token ring search
    // for each iteration we process all moves available
    // from each slot (ie: Ins, IntraSw, InterSw)
    // when there are no move moves we terminate
    // or if we reach stagnation. Stagnation is N moves
    // without improving the bestSolution
    improvedBest  = doNeighborhoodMoves( Ins,     500 );
    improvedBest |= doNeighborhoodMoves( IntraSw, 300 );
    improvedBest |= doNeighborhoodMoves( InterSw, 300 );

    if ( improvedBest ) lastImproved = 0;
    else ++lastImproved;

    std::cout << "TABUSEARCH: Finished iteration: " << currentIteration
              << ", improvedBest: " << improvedBest
              << ", run time: " << start.duration()
              << std::endl;

    STATS->set( "0 Iteration", currentIteration );
    STATS->set( "0 Best Cost After", bestSolution.getCost() );
    dumpStats();
    std::cout << "--------------------------------------------\n";
  }

  //while (improvedBest and ++currentIteration < maxIteration);
  while ( lastImproved < 1 and ++currentIteration < maxIteration );

  std::cout << "TABUSEARCH: Total time: " << start.duration() << std::endl;
}



void TabuSearch::generateNeighborhood( neighborMovesName whichNeighborhood,
                                       std::deque<Move> &neighborhood, const Move &lastMove ) const
{

  // generate the a move neighborhood based on the currentSolution
  // this is fast and efficient as move generation accounts for most
  // of the computational time.
  // We only look at a very small percentage of the actual moves
  // but we have to calcuate savings and feasiblity on all of them

  std::cout << "generateNeighborhood: lastMove: "; lastMove.dump();

  Timer timeNeighboorhoodGeneration;

  switch ( whichNeighborhood ) {
  case Ins:
    ++currentIterationIns;
    //currentSolution.v_getInsNeighborhood(neighborhood, factor);
    currentSolution.getInsNeighborhood( neighborhood, lastMove );

    // collect stats
    generateNeighborhoodStats( "Ins",
                               timeNeighboorhoodGeneration.duration(),
                               neighborhood.size() );
    break;

  case IntraSw:
    ++currentIterationIntraSw;
    //currentSolution.v_getIntraSwNeighborhood(neighborhood, factor);
    currentSolution.getIntraSwNeighborhood( neighborhood, lastMove );

    // collect stats
    generateNeighborhoodStats( "IntraSw",
                               timeNeighboorhoodGeneration.duration(),
                               neighborhood.size() );
    break;

  case InterSw:
    ++currentIterationInterSw;
    //currentSolution.v_getInterSwNeighborhood(neighborhood, factor);
    currentSolution.getInterSwNeighborhood( neighborhood, lastMove );

    // collect stats
    generateNeighborhoodStats( "InterSw",
                               timeNeighboorhoodGeneration.duration(),
                               neighborhood.size() );
    break;
  }
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

bool TabuSearch::doNeighborhoodMoves( neighborMovesName whichNeighborhood,
                                      int maxStagnation )
{
  bool improvedBest = false;
  int stagnationCnt = 0;
  double factor = 0.5;
  std::string mName;

  switch ( whichNeighborhood ) {
  case Ins:     mName = "Ins";    break;

  case IntraSw: mName = "IntraSw"; break;

  case InterSw: mName = "InterSw"; break;
  }

  // we always start from the best solution of the last run [article]
  currentSolution = bestSolution;

  STATS->set( "factor", factor );

  std::deque<Move> neighborhood;
  Move lastMove;

  do {
    int currentMoveIteration;

    switch ( whichNeighborhood ) {
    case Ins:     currentMoveIteration = currentIterationIns; break;

    case IntraSw: currentMoveIteration = currentIterationIntraSw; break;

    case InterSw: currentMoveIteration = currentIterationInterSw; break;
    }

    // generate or regenerate the neighborhood moves
    generateNeighborhood( whichNeighborhood, neighborhood, lastMove );

    // and sort it so we can work from the best to the worst
    std::sort( neighborhood.begin(), neighborhood.end(), Move::bySavings );

    // dump the neighborhood
    //        for (int i=0;i<neighborhood.size();i++) neighborhood[i].dump();
    //        std::cout<<"======================================================";

    // take the best move that we may apply and apply it, if any
    Timer applyMoveTimer;
    bool allTabu = true;

    for ( std::deque<Move>::iterator it = neighborhood.begin();
          it != neighborhood.end(); ++it ) {

      // if the move is aspirational then we apply it
      if ( currentSolution.getCost() - it->getsavings() < bestSolutionCost ) {
        std::cout << "doNeighborhoodMoves[" << currentMoveIteration <<
                  "]: Aspiration move: "; it->dump();
        currentSolution.applyMove( *it );
        makeTabu( *it );
        lastMove = *it;

        bestSolution = currentSolution;
        bestSolutionCost = bestSolution.getCost();
        improvedBest = true;
        stagnationCnt = 0;
        allTabu = false;
        STATS->set( "best Updated Last At", currentIteration );
        STATS->inc( "best Updated Cnt" );
        STATS->inc( "cnt Applied " + mName );

        // ok we made a move, so now the neighborhood is no
        // longer valid so break to regenerate a new neighborhood
        break;
      }
      // if the move is not Tabu, then we apply it even if
      // it makes the solution worse so we move to a new
      // area of the search space
      else if ( ! isTabu( *it ) ) {
        std::cout << "doNeighborhoodMoves[" << currentMoveIteration << "]: Not Tabu: ";
        it->dump();
        currentSolution.applyMove( *it );
        makeTabu( *it );
        lastMove = *it;

        allTabu = false;
        STATS->inc( "cnt Applied " + mName );

        // ok we made a move, so now the neighborhood is no
        // longer valid so break to regenerate a new neighborhood
        break;
      } else {
        ++stagnationCnt;
      }
    }

    // if all the moves are tabu then we need to pick one
    // the articles says to pick the best
    if ( allTabu and neighborhood.size() ) {
      int pick = 0; // pick the best

      lastMove = *( neighborhood.begin() + pick );
      currentSolution.applyMove( lastMove );
      makeTabu( lastMove );

      std::cout << "doNeighborhoodMoves[" << currentMoveIteration << "]: All Tabu(" <<
                pick << "): "; lastMove.dump();
      STATS->inc( "cnt Applied " + mName );
    }

    STATS->addto( "time Apply Moves", applyMoveTimer.duration() );
  } while ( stagnationCnt < maxStagnation );

  if ( not improvedBest )
    std::cout << "Stagnation reached in neighborhood: " << mName << std::endl;

  return improvedBest;
}

