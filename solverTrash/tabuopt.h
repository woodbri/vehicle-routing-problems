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
#include <cstdlib>

#include "vrp_assert.h"
#include "logger.h"
#include "stats.h"
#include "timer.h"
#include "move.h"
#include "optsol.h"
#include "tabubase.h"

class TabuOpt: public TabuBase<OptSol> {

  public:
    TabuOpt( const OptSol &initialSolution ) :
        TabuBase( initialSolution ) {
        #ifdef DOSTATS
        STATS->inc( "TabuOpt::TabuOpt" );
        #endif
        computeCosts( bestSolution );
        Timer start;
        #ifndef LOG
        bestSolution.tau();
        #endif
        bestSolution.optimizeTruckNumber();
        bestTabuList.clear();
        computeCosts( bestSolution );
        bestSolutionCost = bestSolution.getCost();
        setBestAsCurrent();
        #ifndef LOG
        DLOG( INFO ) << "TABUSEARCH: Removal of truck time: " << start.duration();
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
    };



    void optimizeTruckNumber();

    void search();
    bool doNeighborhoodMoves( Move::Mtype whichNeighborhood, int maxCnt,
                              Move::Mtype mtype );
    void getNeighborhood( Move::Mtype whichNeighborhood, Moves &neighborhood ,
                          double factor, Move::Mtype mtype ) const;
    bool applyAspirationalTabu( Moves &aspirationalTabu );
    bool classifyMoves( Moves &neighborhood );
    bool applyNonTabu ( Moves &moves );
    bool applyAmove ( const Move &move );
    bool applyMoves ( std::string type, Moves &moves );
    bool applyAspirationalNotTabu ( const Move &move );
    bool applyTabu ( Moves &moves );
    bool applyTabu ( Moves &moves, int strategy );
    bool computeCosts( OptSol &s ) ;
    bool reachedMaxCycles( int, Move::Mtype );
    bool dumpMoves( std::string str, Moves moves ) const ;
    void cleanUpInterSwMoves( Moves &moves, const Move &guide ) const ;
    void cleanUpIntraSwMoves( Moves &moves, const Move &guide ) const ;
    void cleanUpInsMoves( Moves &moves, const Move &guide, bool &reverseFound ) ;
    void cleanUpMoves( const Move guide ) ;


    bool inRange( int center, int data, int step ) const;
  private:
    int limitIntraSw;
    int limitInterSw;
    int limitIns;

    //mutable Moves neighborhood;
    mutable Moves aspirationalNotTabu;
    mutable Moves aspirationalTabu;
    mutable Moves notTabu;
    mutable Moves tabu;
    mutable Moves reverseMoves;

};

#endif

