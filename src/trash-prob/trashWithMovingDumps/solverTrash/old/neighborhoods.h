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
#ifndef NEIGHBORHOODS_H
#define NEIGHBORHOODS_H

#include "solution.h"
#include "move.h"

class Neighborhoods : public Solution
{

public:

  Neighborhoods( const Solution &solution ): Solution( solution ) {};

  bool isNotFeasible( const Move &m ) const ;
  double getMoveSavings( const Move &m ) const ;
  int clearRelatedMoves( std::deque<Move> &moves, const Move &lastMove )  const;
  int addRelatedMovesIns( std::deque<Move> &moves, const Move &lastMove )  const;
  int addRelatedMovesIntraSw( std::deque<Move> &moves,
                              const Move &lastMove )  const;
  int addRelatedMovesInterSw( std::deque<Move> &moves,
                              const Move &lastMove )  const;
  void getInsNeighborhood( std::deque<Move> &moves, const Move &lastMove ) const ;
  void getIntraSwNeighborhood( std::deque<Move> &moves,
                               const Move &lastMove ) const;
  void getInterSwNeighborhood( std::deque<Move> &moves,
                               const Move &lastMove ) const;
  void applyMove( const Move &m );


};

#endif
