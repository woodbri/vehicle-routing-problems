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

#include "trashstats.h"
#include "timer.h"
#include "move.h"
#include "optsol.h"

class TabuBase {


  protected:
    typedef enum { Ins, IntraSw, InterSw } neighborMovesName;
    std::map<const Move, int> TabuList;

    int tabuLengthIns;
    int tabuLengthIntraSw;
    int tabuLengthInterSw;

    bool allTabu;
    Move bestMoveAllTabu;

    int maxIteration;
    int currentIteration;
    mutable int currentIterationIns;
    mutable int currentIterationIntraSw;
    mutable int currentIterationInterSw;

    //OptSol bestSolution;
    //OptSol currentSolution;

    //double bestSolutionCost;


  public:
    TabuBase(/*const OptSol &initialSolution*/)// :
//        bestSolution(initialSolution), currentSolution(initialSolution)
    {
#ifdef VICKY
//        bestSolution.v_computeCosts();
#else
//        bestSolution.computeCosts();
#endif
//        bestSolution.dump();
//        bestSolutionCost = bestSolution.getCost();
        currentIteration = currentIterationIns = currentIterationIntraSw = currentIterationInterSw = 0;
        maxIteration = 1000;
        srand(37);
/*        int ncnt = initialSolution.getNodeCount() / 5;
        tabuLengthIns     = std::max(5, std::min(ncnt, 40));
        tabuLengthIntraSw = std::max(5, std::min(ncnt, 15));
        tabuLengthInterSw = std::max(5, std::min(ncnt, 10));

        STATS->set("tabu Length Ins", tabuLengthIns);
        STATS->set("tabu Length IntraSw", tabuLengthIntraSw);
        STATS->set("tabu Length InterSw", tabuLengthInterSw);

        // for repeatible results set this to a constant value
        // for more random results use: srand( time(NULL) );
	limitIntraSw=bestSolution.getFleetSize();
	limitInterSw=limitIntraSw*(limitIntraSw-1) ;
	limitIns=limitIntraSw ;
*/    };

    void dumpTabuList() const;
    void dumpStats() const;
    void makeTabu(const Move &m);
    void cleanExpired();
    bool isTabu(const Move& m) const;
//    Solution getBestSolution() const { return bestSolution; };
//    Solution getCurrentSolution() const {return currentSolution; };

    int getCurrentIteration() const { return currentIteration; };
    int getMaxIteration() const { return maxIteration; };
    int getTabuLengthIns() const { return tabuLengthIns; };
    int getTabuLengthIntraSw() const { return tabuLengthIntraSw; };
    int getTabuLengthInterSw() const { return tabuLengthInterSw; };


    void setMaxIteration(int n) { assert(n>0); maxIteration = n; };
    void settabuLengthIns(int n) { assert(n>0); tabuLengthIns = n; };
    void settabuLengthIntraSw(int n) { assert(n>0); tabuLengthIntraSw = n; };
    void settabuLengthInterSw(int n) { assert(n>0); tabuLengthInterSw = n; };
    void generateNeighborhoodStats(std::string mtype, double tm, int cnt) const;
    void addToStats (const Move &move) const ;
    void savingsStats (const Move &move) const;

#ifdef TESTED
    void search();
    void generateNeighborhood(neighborMovesName whichNeighborhood, std::deque<Move>& neighborhood, const Move& lastMove) const;
    bool doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxStagnation);


    void v_search();
    bool v_doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxCnt, std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu, std::deque<Move> &tabu);
    void v_getNeighborhood(neighborMovesName whichNeighborhood,std::deque<Move> &neighborhood,double factor) const;
    bool v_applyAspirationalTabu(std::deque<Move> &aspirationalTabu);
    bool v_applyAspirational(std::deque<Move> &neighborhood, std::deque<Move> &notTabu,std::deque<Move> &tabu);
    bool v_applyAspirationalNotTabu(std::deque<Move> &neighborhood, std::deque<Move> &aspirationalTabu,std::deque<Move> &notTabu,std::deque<Move> &tabu);
    bool v_applyNonTabu (std::deque<Move> &notTabu);
    bool v_applyTabu (std::deque<Move> &tabu);
    bool v_applyTabu (std::deque<Move> &tabu, int strategy);
    void v_computeCosts(OptSol &s) ;
    bool reachedMaxCycles(int,neighborMovesName);
#endif
    private:
	int limitIntraSw;
	int limitInterSw;
	int limitIns;

};

#endif

