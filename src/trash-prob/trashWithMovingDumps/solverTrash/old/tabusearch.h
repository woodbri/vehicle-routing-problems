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
#ifndef TABUSEARCH_H
#define TABUSEARCH_H

#include <map>
#include <cassert>
#include <cstdlib>

#include "stats.h"
#include "timer.h"
#include "move.h"
#include "neighborhoods.h"
#include "tabubase.h"

class TabuSearch : public TabuBase<Neighborhoods>
{

public:
  TabuSearch( const Neighborhoods initialSolution ) :
    TabuBase( initialSolution ) {
    bestSolution.computeCosts();
    bestSolution.dump();
    bestSolutionCost = bestSolution.getCost();
  };

  void search();
  void generateNeighborhood( neighborMovesName whichNeighborhood,
                             std::deque<Move> &neighborhood, const Move &lastMove ) const;
  bool doNeighborhoodMoves( neighborMovesName whichNeighborhood,
                            int maxStagnation );

};

#endif

