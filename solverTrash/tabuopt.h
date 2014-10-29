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
#ifndef TABUOPT_H
#define TABUOPT_H

#include <map>
#include <cassert>
#include <cstdlib>

#include "trashstats.h"
#include "timer.h"
#include "move.h"
#include "optsol.h"
#include "tabubase.h"

class TabuOpt: public TabuBase<OptSol> {

  public:
    TabuOpt(const OptSol &initialSolution) :
         TabuBase(initialSolution)
    {
        computeCosts(bestSolution);
        bestSolutionCost = bestSolution.getCost();
	limitIntraSw=bestSolution.getFleetSize();
	limitInterSw=limitIntraSw*(limitIntraSw-1) ;
	limitIns    =limitInterSw ;
        STATS->set("limitIntraSw", limitIntraSw);
        STATS->set("limitInterSw", limitIntraSw);
        STATS->set("limitIns", limitIntraSw);
    };



    void optimizeTruckNumber();

    void search();
    bool doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxCnt, std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu, std::deque<Move> &tabu);
    void getNeighborhood(neighborMovesName whichNeighborhood,std::deque<Move> &neighborhood,double factor) const;
    bool applyAspirationalTabu(std::deque<Move> &aspirationalTabu);
    bool applyAspirational(std::deque<Move> &neighborhood, std::deque<Move> &notTabu,std::deque<Move> &tabu);
    bool applyAspirationalNotTabu(std::deque<Move> &neighborhood, std::deque<Move> &aspirationalTabu,std::deque<Move> &notTabu,std::deque<Move> &tabu);
    bool applyNonTabu (std::deque<Move> &notTabu);
    bool applyTabu (std::deque<Move> &tabu);
    bool applyTabu (std::deque<Move> &tabu, int strategy);
    void computeCosts(OptSol &s) ;
    bool reachedMaxCycles(int,neighborMovesName);

    private:
	int limitIntraSw;
	int limitInterSw;
	int limitIns;

};

#endif

