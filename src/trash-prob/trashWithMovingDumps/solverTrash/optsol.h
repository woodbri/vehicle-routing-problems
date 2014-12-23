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
#ifndef OPTSOL_H
#define OPTSOL_H

#include "solution.h"
#include "move.h"

class OptSol : public Solution {
    typedef std::set<Move, Move::compMove> Moves;
    typedef std::set<Move, Move::compMove>::iterator MovesItr;


  public:

    OptSol( const Solution &solution ): Solution( solution ) {
        intraTruckPos = 0;
        interTruckPos1 = 0;
        interTruckPos2 = 1;
        insTruckPos1 = fleet.size() - 1;
        insTruckPos2 = 0;
    };

    void getIntraSwNeighborhood( Moves &moves) const;
    void getInsNeighborhood( Moves &moves  ) const ;
    void getInterSwNeighborhood( Moves &moves, double factor ) const;

    bool v_applyInterSwMove( const Move &move );
    bool testInterSwMove( const Move &move ) const;
    bool testInsMove( const Move &move ) const;
    void v_applyMove( const Move & );
    void optimizeTruckNumber();
    void setFreeSpaces();

  private:
    bool emptyAtruck( std::deque<int> from, std::deque<int> toThisOnes );
    bool emptyTheTruck( POS fromTruck, std::deque<int> toThisOnes );

    mutable POS intraTruckPos;
    mutable POS interTruckPos1;
    mutable POS interTruckPos2;
    mutable POS insTruckPos1;
    mutable POS insTruckPos2;


};

#endif
