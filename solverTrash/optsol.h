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

public:

OptSol(const Solution &solution): Solution(solution){
	intraTruckPos=0;
	interTruckPos1=0;
	interTruckPos2=1;
	insTruckPos1=fleet.size()-1;
	insTruckPos2=0;
 }; 

    void v_getIntraSwNeighborhood(std::deque<Move>& moves, double factor) const;
    void v_getInsNeighborhood(std::deque<Move>& moves,double factor) const ;
    void v_getInsNeighborhood(std::deque<Move>& moves,double factor, int count) const;
    void v_getInterSwNeighborhood(std::deque<Move>& moves, double factor) const;

    bool v_applyInsMove( const Move &move);
    void v_applyMove(const Move&);
    void optimizeTruckNumber();
    bool emptyTheTruck(int fromTruck, std::deque<int> toThisOnes);

private:
   mutable int intraTruckPos;
   mutable int interTruckPos1;
   mutable int interTruckPos2;
   mutable int insTruckPos1;
   mutable int insTruckPos2;

};

#endif
