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
        Timer start;
bestSolution.tau();
    	bestSolution.optimizeTruckNumber();
    	bestTabuList.clear();
        setBestAsCurrent();
currentSolution.tau();
    	//currentSolution=bestSolution;
    	std::cout << "TABUSEARCH: Removal of truck time: " << start.duration() << std::endl;
bestSolution.tau();

        computeCosts(bestSolution);
        bestSolutionCost = bestSolution.getCost();
	limitIntraSw=bestSolution.getFleetSize();
	limitInterSw=limitIntraSw*(limitIntraSw-1)/2 ;
	limitIns    =limitInterSw ;
        STATS->set("limitIntraSw", limitIntraSw);
        STATS->set("limitInterSw", limitIntraSw);
        STATS->set("limitIns", limitIntraSw);
    };



    void optimizeTruckNumber();

    void search();
    bool doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxCnt);
    void getNeighborhood(neighborMovesName whichNeighborhood, Moves &neighborhood , double factor) const;
    bool applyAspirationalTabu(Moves &aspirationalTabu);
    bool classifyMoves(Moves &neighborhood);
    bool applyNonTabu (Moves &moves);
    bool applyAmove (const Move &move);
    bool applyMoves (std::string type, Moves &moves);
    bool applyAspirationalNotTabu (const Move &move);
    bool applyTabu (Moves &moves);
    bool applyTabu (Moves &moves, int strategy);
    bool computeCosts(OptSol &s) ;
    bool reachedMaxCycles(int,neighborMovesName);
    bool dumpMoves(std::string str, Moves moves) const ;
    void cleanUpInterSwMoves(Moves &moves, const Move &guide) const ;
    void cleanUpInsMoves(Moves &moves, const Move &guide) ;
    void cleanUpMoves(const Move &guide) ;
#ifndef TESTED
    void compareCostWithOSRM(Moves &neighborhood);
#endif
    private:
	int limitIntraSw;
	int limitInterSw;
	int limitIns;
	//mutable Moves neighborhood;
	mutable Moves aspirationalNotTabu;
	mutable Moves aspirationalTabu;
	mutable Moves notTabu;
	mutable Moves tabu;

};

#endif

