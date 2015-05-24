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
#include "signalhandler.h"

#include "pg_types_vrp.h"
#include "tabubase.h"

class TabuOpt: public TabuBase<OptSol>
{
private:
  typedef unsigned long int UID;
  typedef unsigned long int POS;
  typedef unsigned long int UINT;


public:
  TabuOpt( const OptSol &initialSolution, unsigned int iteration );

  void optimizeTruckNumber();

  void search();
  bool doNeighborhoodMoves( Move::Mtype whichNeighborhood, int maxCnt);
  void getNeighborhood( Move::Mtype whichNeighborhood, Moves &neighborhood ,
                        double factor ) const;
  bool applyAspirationalTabu( Moves &aspirationalTabu );
  bool classifyMoves( Moves &neighborhood );
  bool applyNonTabu ( Moves &moves );
  bool applyAmove ( const Move &move );
  bool applyMoves ( const std::string &type, Moves &moves );
  bool applyAspirationalNotTabu ( const Move &move );
  bool applyTabu ( Moves &moves );
  bool applyTabu ( Moves &moves, int strategy );
  bool computeCosts( OptSol &s ) ;
  bool reachedMaxCycles( int, Move::Mtype );
  void dumpMoves( std::string str, Moves moves ) const ;
  void cleanUpInterSwMoves( Moves &moves, const Move &guide ) const ;
  void cleanUpIntraSwMoves( Moves &moves, const Move &guide ) const ;
  void cleanUpInsMoves( Moves &moves, const Move &guide, bool &reverseFound ) ;
  void cleanUpMoves( const Move guide ) ;
  vehicle_path_t *getSolutionForPg( UINT &count ) const ;


  void clean() {
    TabuBase<OptSol>::clear();
    aspirationalNotTabu.clear();
    aspirationalTabu.clear();
    notTabu.clear();
    tabu.clear();
    reverseMoves.clear();
  }

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

