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
#ifndef TABUBASE_H
#define TABUBASE_H

#include <map>
#include <cstdlib>
#include <sstream>

#include "vrp_assert.h"

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "stats.h"
#include "timer.h"
#endif

#include "move.h"
#include "optsol.h"

template <class Ksolution>
class TabuBase
{


protected:
  typedef unsigned long int POS;
  typedef unsigned long int UID;
  typedef std::set<Move, Move::compMove> Moves;
  typedef std::set<Move, Move::compMove>::iterator MovesItr;


  std::map<const Move, int> TabuList;

private:
  int tabuLengthIns;
  int tabuLengthIntraSw;
  int tabuLengthInterSw;

  int totalMovesMade;   // Total times makeTabu is called

protected:
  bool allTabu;
  //Move bestMoveAllTabu;

  int maxIteration;     // limit for token ring cycles
  int currentIteration; // counter for token ring cycles

  int limitIntraSw;
  int limitInterSw;
  int limitIns;

  Ksolution bestSolution;
  double bestSolutionCost;
  std::map<const Move, int> bestTabuList;

private:
  int bestIterationIns;
  int bestIterationIntraSw;
  int bestIterationInterSw;


protected:
  Ksolution currentSolution;
  double currentCost;
private:
  int currentIterationIns;
  int currentIterationIntraSw;
  int currentIterationInterSw;



public:

  void  clear() {
    currentSolution.clear();
    bestSolution.clear();
    bestTabuList.clear();
    TabuList.clear();
  }




  TabuBase( const Ksolution &initialSolution ) :
    bestSolution( initialSolution ), currentSolution( initialSolution ) {
    currentIteration = currentIterationIns = currentIterationIntraSw =
                         currentIterationInterSw = 0;
    bestIterationIns = bestIterationIntraSw = bestIterationInterSw = 0;
    totalMovesMade = 0;
    maxIteration = 1000;
    int ncnt = initialSolution.getNodeCount() / 5;
    tabuLengthIns     = std::max( 5, std::min( ncnt, 40 ) );
    tabuLengthIntraSw = std::max( 5, std::min( ncnt, 15 ) );
    tabuLengthInterSw = std::max( 5, std::min( ncnt, 10 ) );
#ifdef DOSTATS
    STATS->set( "tabu Length Ins", tabuLengthIns );
    STATS->set( "tabu Length IntraSw", tabuLengthIntraSw );
    STATS->set( "tabu Length InterSw", tabuLengthInterSw );
#endif
    // for repeatible results set this to a constant value
    // for more random results use: srand( time(NULL) );
    srand( 37 );
  };

  void setCurrentAsBest() {
#ifdef DOSTATS
    STATS->inc( "Tabubase::setCurrentAsBest" );
#endif
    bestSolution = currentSolution;
    bestSolutionCost = currentCost;
    bestIterationIns = currentIterationIns;
    bestIterationIntraSw = currentIterationIntraSw;
    bestIterationInterSw = currentIterationInterSw;
    bestTabuList = TabuList;
  };

  void setBestAsCurrent() {
#ifdef DOSTATS
    STATS->inc( "Tabubase::setBestAsCurrent" );
#endif
    currentSolution = bestSolution;
    currentCost = bestSolutionCost;
    currentIterationIns = bestIterationIns;
    currentIterationIntraSw = bestIterationIntraSw;
    currentIterationInterSw = bestIterationInterSw;
    TabuList = bestTabuList;

  };



  Solution getBestSolution() const { return bestSolution; };
  Solution getCurrentSolution() const {return currentSolution; };

  int getCurrentIteration() const { return currentIteration; };
  int getTotalMovesMade() const { return totalMovesMade; };
  void getCurrentCounters( int &currIterIns, int &currIterIntra,
                           int &currIterInter ) {
    currIterIns = currentIterationIns;
    currIterIntra = currentIterationIntraSw;
    currIterInter = currentIterationInterSw;
  };
  void getBestCounters( int &currIterIns, int &currIterIntra,
                        int &currIterInter ) {
    currIterIns = bestIterationIns;
    currIterIntra = bestIterationIntraSw;
    currIterInter = bestIterationInterSw;
  };

  int getMaxIteration() const { return maxIteration; };
  int getTabuLengthIns() const { return tabuLengthIns; };
  int getTabuLengthIntraSw() const { return tabuLengthIntraSw; };
  int getTabuLengthInterSw() const { return tabuLengthInterSw; };


  void setMaxIteration( int n ) { assert( n > 0 ); maxIteration = n; };
  void setTabuLengthIns( int n ) { assert( n > 0 ); tabuLengthIns = n; };
  void setTabuLengthIntraSw( int n ) { assert( n > 0 ); tabuLengthIntraSw = n; };
  void setTabuLengthInterSw( int n ) { assert( n > 0 ); tabuLengthInterSw = n; };

  /*
      bool insForbidden( int truckPos ) {
      std::set<int> list= tabuedForInsInsertionPart();
      return ( list.find(truckPos)==list.end() );
      };


      std::set<int> tabuedForInsInsertionPart() const {
      #ifdef DOSTATS
          STATS->inc("Tabubase::tabuedForInsInsertionPart");
      #endif
      std::set<int> list;
          std::map<const Move, int>::const_iterator it;
      Move move;
          for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
          move=it->first;
          list.insert(move.getInsFromTruck());
      }
  dumpSet("Tabued for insertion",list);
      return list;
      };
  */

  void dumpSet( std::string title, std::set<int> info ) const {
#ifdef DOVRPLOG
    std::stringstream ss;

#ifdef DOSTATS
    STATS->inc( "Tabubase::dumpTabuList" );
#endif
    std::set<int>::const_iterator it;
    ss << "Title: ";

    for ( it = info.begin(); it != info.end(); ++it ) {
      ss << ( *it ) << " ";
    }

    DLOG( INFO ) << ss.str();
#endif
  };




  void dumpTabuList() const {
#ifdef DOVRPLOG
    std::stringstream ss;

#ifdef DOSTATS
    STATS->inc( "Tabubase::dumpTabuList" );
#endif

    std::map<const Move, int>::const_iterator it;

    ss << "TabuList at iteration: " << currentIteration << "\t";
    ss << "total Moves made: " << totalMovesMade << "\n";

    for ( it = TabuList.begin(); it != TabuList.end(); ++it ) {
      it->first.dump();
      ss << " - expires: " << it->second << "\n";
    }

    ss << "--------------------------";
    DLOG( INFO ) << ss.str();
#endif
  };


  void dumpStats() const {
#ifdef DOSTATS
    STATS->inc( "Tabubase::dumpStats" );
#ifdef VRPMINTRACE
    DLOG( INFO ) << "TabuList Stats at iteration: " << currentIteration;
#endif
    STATS->dump( "" );
#endif
  };


  void generateNeighborhoodStats( std::string mtype, double tm, int cnt ) const {
#ifdef DOSTATS
    STATS->inc( "Tabubase::generateNeighborhoodStats" );
    STATS->addto( "time Gen " + mtype, tm );
    STATS->inc( "cnt Calls Gen " + mtype );
    STATS->addto( "cum Moves " + mtype, cnt );
#endif
#ifdef VRPMAXTRACE
    DLOG( INFO ) << "doNeighborhoodMoves for " << mtype << ": " << cnt
                 << " moves generated";
#endif
  };



  bool isTabu( const Move &move_e ) const {
#ifdef DOSTATS
    STATS->inc( "Tabubase::isTabu" );
#endif

    std::map<const Move, int>::const_iterator it;
    Move tabu;
    int expires;

    for ( it = TabuList.begin(); it != TabuList.end(); ++it ) {
      tabu = ( it->first );
      expires = it->second;

      //skipping expiered moves (just in case they were not cleared
      if ( ( tabu.getmtype() == Move::Ins and expires < currentIterationIns )
           or ( tabu.getmtype() == Move::IntraSw and expires < currentIterationIntraSw )
           or ( tabu.getmtype() == Move::InterSw and expires < currentIterationInterSw )
         ) continue;

      if ( tabu.isTabu( move_e, 6 ) ) {
#ifdef DOSTATS
        STATS->inc( "tabu Moves resulted" );
#endif
        return true;
      }
    }

    return false;
  };







  void cleanExpired() {
#ifdef DOSTATS
    STATS->inc( "Tabubase::cleanExpired" );
#endif
    std::map<const Move, int>::iterator it;
    std::map<const Move, int> oldTabuList = TabuList;

    Move move;
    int expires;
    TabuList.clear();

    for ( it = oldTabuList.begin(); it != oldTabuList.end(); it++ ) {
      move = it->first;
      expires = it->second;

      if ( ( it->first.isIns()
             and expires <= currentIterationIns ) ) continue;
      else if ( ( move.isIntraSw()
                  and expires <= currentIterationIntraSw ) ) continue;
      else if ( ( move.isInterSw()
                  and expires <= currentIterationInterSw ) ) continue;
      else TabuList[move] = expires;
    }

  }


  void makeTabu( const Move &move ) {
#ifdef DOSTATS
    STATS->inc( "Tabubase::makeTabu" );
#endif
    // generate a randon value between -2 and +2
    // to adjust the tabu length with
    totalMovesMade++;
    int r = rand() % 5 - 2;

    switch ( move.getmtype() ) {
    case Move::Ins:
      currentIterationIns++;
      TabuList[move] = currentIterationIns + tabuLengthIns + r;
#ifdef DOSTATS
      STATS->inc( "tabu Ins Moves Added" );
#endif
      break;

    case Move::IntraSw:
      currentIterationIntraSw++;
      TabuList[move] = currentIterationIntraSw + tabuLengthIntraSw + r;
#ifdef DOSTATS
      STATS->inc( "tabu IntraSw Moves Added" );
#endif
      break;

    case Move::InterSw:
      currentIterationInterSw++;
      TabuList[move] = currentIterationInterSw + tabuLengthInterSw + r;
#ifdef DOSTATS
      STATS->inc( "tabu InterSw Moves Added" );
#endif
      break;
    }

    cleanExpired();
#ifdef DOSTATS
    addToStats( move );
    savingsStats( move );
#endif
  }


  void savingsStats( const Move &move ) const {
#ifdef DOSTATS
    STATS->inc( "Tabubase::savingsStats" );

    if ( move.getsavings() < 0 ) {
      STATS->inc( "neg savings applied" );

      switch  ( move.getmtype() ) {
      case Move::Ins:
        STATS->inc( "neg sav Ins applied" ); break;

      case Move::IntraSw:
        STATS->inc( "neg sav IntraSw applied" ); break;

      case Move::InterSw:
        STATS->inc( "neg sav InterSw applied" ); break;
      }
    } else {
      STATS->inc( "pos savings applied" );

      switch  ( move.getmtype() ) {
      case Move::Ins:
        STATS->inc( "pos sav Ins applied" ); break;

      case Move::IntraSw:
        STATS->inc( "pos sav IntraSw applied" ); break;

      case Move::InterSw:
        STATS->inc( "pos sav InterSw applied" ); break;
      }
    }

#endif
  };



  void addToStats( const Move &move ) const {
#ifdef DOSTATS
    STATS->inc( "Tabubase::removeTruckFromTabuList" );

    switch ( move.getmtype() ) {
    case Move::Ins:     STATS->inc( "cnt Ins Applied" );    break;

    case Move::IntraSw: STATS->inc( "cnt IntraSw Applied" ); break;

    case Move::InterSw: STATS->inc( "cnt InterSw Applied" ); break;
    }

#endif
  };


  void removeTruckFromTabuList( POS truckPos ) {
#ifdef DOSTATS
    STATS->inc( "Tabubase::removeTruckFromTabuList" );
#endif
#ifdef VRPMAXTRACE
    DLOG( INFO ) << "Entering TabuBase::removeTruckFromTabuList";
#endif
    int pos1, pos2;
    Move move;
    int expires;
    std::map<const Move, int> oldTabuList = TabuList;
    TabuList.clear();

    for ( std::map<Move, int>::iterator it = oldTabuList.begin();
          it != oldTabuList.end(); ++it ) {
      pos1 = it->first.getvid1();
      pos2 = it->first.getvid2();
      move = it->first;
      expires = it->second;

      if ( pos1 == truckPos or pos2 == truckPos ) continue;

      //interface for position is with vid
      if  ( pos1 > truckPos )  move.setvid1( pos1 - 1 );

      //interface for position is with vid
      if  ( pos2 > truckPos )  move.setvid2( pos2 - 1 );

      TabuList[move] = expires;
    }
  }





};

#endif

