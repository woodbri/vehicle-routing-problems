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
#include <cassert>
#include <cstdlib>

#include "trashstats.h"
#include "timer.h"
#include "move.h"
#include "optsol.h"

template <class Ksolution>
class TabuBase  {


  protected:
    typedef enum { Ins, IntraSw, InterSw } neighborMovesName;
    typedef unsigned long int POS;
    typedef unsigned long int UID;

    std::map<const Move, int> TabuList;

  private:
    int tabuLengthIns;
    int tabuLengthIntraSw;
    int tabuLengthInterSw;

    int totalMovesMade;   // Total times makeTabu is called

  protected:
    bool allTabu;
    Move bestMoveAllTabu;

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
    TabuBase(const Ksolution &initialSolution) :
        bestSolution(initialSolution), currentSolution(initialSolution)
    {
        currentIteration = currentIterationIns = currentIterationIntraSw = currentIterationInterSw = 0;
        bestIterationIns = bestIterationIntraSw = bestIterationInterSw = 0;
	totalMovesMade = 0;
        maxIteration = 1000;
        int ncnt = initialSolution.getNodeCount() / 5;
        tabuLengthIns     = std::max(5, std::min(ncnt, 40));
        tabuLengthIntraSw = std::max(5, std::min(ncnt, 15));
        tabuLengthInterSw = std::max(5, std::min(ncnt, 10));

        STATS->set("tabu Length Ins", tabuLengthIns);
        STATS->set("tabu Length IntraSw", tabuLengthIntraSw);
        STATS->set("tabu Length InterSw", tabuLengthInterSw);

        // for repeatible results set this to a constant value
        // for more random results use: srand( time(NULL) );
        srand(37);
    };

    void setCurentAsBest() {
	bestSolution=currentSolution;
    	bestSolutionCost=currentCost;
     	bestIterationIns=currentIterationIns;
     	bestIterationIntraSw=currentIterationIntraSw;
     	bestIterationInterSw=currentIterationInterSw;
        bestTabuList=TabuList;
    };

    void setBestAsCurrent() {
        currentSolution=bestSolution;
        currentCost=bestSolutionCost;
std::cout<<"currentIterationIns"<<currentIterationIns<<"\n";
std::cout<<"bestIterationIns"<<bestIterationIns<<"\n";
        currentIterationIns=bestIterationIns;
        currentIterationIntraSw=bestIterationIntraSw;
        currentIterationInterSw=bestIterationInterSw;
        TabuList=bestTabuList;
std::cout<<"currentIterationIns"<<currentIterationIns<<"\n";
std::cout<<"bestIterationIns"<<bestIterationIns<<"\n";

    };



    Solution getBestSolution() const { return bestSolution; };
    Solution getCurrentSolution() const {return currentSolution; };

    int getCurrentIteration() const { return currentIteration; };
    int getTotalMovesMade() const { return totalMovesMade; };
    void getCurrentCounters(int &currIterIns, int &currIterIntra, int &currIterInter) {
	currIterIns=currentIterationIns;
	currIterIntra=currentIterationIntraSw;
	currIterInter=currentIterationInterSw;
    };
    void getBestCounters(int &currIterIns, int &currIterIntra, int &currIterInter) {
	currIterIns=bestIterationIns;
	currIterIntra=bestIterationIntraSw;
	currIterInter=bestIterationInterSw;
    };

    int getMaxIteration() const { return maxIteration; };
    int getTabuLengthIns() const { return tabuLengthIns; };
    int getTabuLengthIntraSw() const { return tabuLengthIntraSw; };
    int getTabuLengthInterSw() const { return tabuLengthInterSw; };


    void setMaxIteration(int n) { assert(n>0); maxIteration = n; };
    void setTabuLengthIns(int n) { assert(n>0); tabuLengthIns = n; };
    void setTabuLengthIntraSw(int n) { assert(n>0); tabuLengthIntraSw = n; };
    void setTabuLengthInterSw(int n) { assert(n>0); tabuLengthInterSw = n; };


    bool insForbidden( int truckPos ) {
	std::set<int> list= tabuedForInsInsertionPart();
	return ( list.find(truckPos)==list.end() );
    };
	

    std::set<int> tabuedForInsInsertionPart() const {
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

	
void dumpSet(std::string title, std::set<int> info) const{
        std::set<int>::const_iterator it;
	std::cout<<"Title: ";
        for (it = info.begin(); it!=info.end(); ++it) {
		std::cout<<(*it)<<" ";
        }
	std::cout<<"\n";
};




void dumpTabuList() const {
    std::map<const Move, int>::const_iterator it;

    std::cout << "TabuList at iteration: " << currentIteration <<"\t";
    std::cout << "total Moves made: " << totalMovesMade << std::endl;
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
        it->first.dump();
        std::cout << " - expires: " << it->second << std::endl;
    }
    std::cout << "--------------------------" << std::endl;
};


void dumpStats() const {
    std::cout << "TabuList Stats at iteration: " << currentIteration << std::endl;
    STATS->dump("");
};


void generateNeighborhoodStats(std::string mtype, double tm, int cnt) const {
    STATS->addto("time Gen "+mtype, tm);
    STATS->inc("cnt Calls Gen "+mtype);
    STATS->addto("cum Moves "+mtype, cnt);
std::cout << "\tdoNeighborhoodMoves for " << mtype << ": " << cnt
<< " moves generated" << std::endl;
};



bool isTabu(const Move& m) const {
    std::map<const Move, int>::const_iterator it;

    bool myTabu=v_isTabu(m);
    STATS->inc("tabu Moves Checked");
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
        //if (it->second < currentIteration) continue;
        if ( (it->first.getmtype()==Move::Ins and
              it->second < currentIterationIns) or
             (it->first.getmtype()==Move::IntraSw and
              it->second < currentIterationIntraSw) or
             (it->first.getmtype()==Move::InterSw and
              it->second < currentIterationInterSw)
             ) continue;
        if (it->first.getmtype() != m.getmtype() ) continue;
        if (m.isForbidden(it->first)) {
m.Dump();
it->first.Dump();
	    assert (myTabu==true);
            STATS->inc("tabu Moves Checked Tabu");
            return true;
        }
    }
    assert (myTabu==false);
    return false;
};


bool v_isTabu(const Move &move_e) const {

    std::map<const Move, int>::const_iterator it;
    Move tabu;
    int expires;
    STATS->inc("tabu Moves Checked");
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
	tabu=(it->first);
	expires= it->second;

        //skipping expiered moves (just in case they were not cleared
        if ( (tabu.getmtype()==Move::Ins and expires < currentIterationIns) 
             or (tabu.getmtype()==Move::IntraSw and expires < currentIterationIntraSw) 
             or (tabu.getmtype()==Move::InterSw and expires < currentIterationInterSw)
             ) continue;

        if (tabu.isTabu(move_e,6)) {
            STATS->inc("tabu Moves resulted");
            return true;
        }
    }
    return false;
};







void cleanExpired() {
    std::map<const Move, int>::iterator it;
    std::map<const Move, int> oldTabuList=TabuList;
    Move move;
    int expires;
    TabuList.clear();
    for (it = oldTabuList.begin(); it!=oldTabuList.end(); it++) {
	move = it->first;
	expires= it->second;
	if ( (move.isIns() and expires > currentIterationIns)
	   or (move.isIntraSw() and expires > currentIterationIntraSw)
	   or (move.isInterSw() and expires > currentIterationInterSw) )
		TabuList[move]=expires;
    }
}


void makeTabu(const Move &m) {
    // generate a randon value between -2 and +2
    // to adjust the tabu length with
    totalMovesMade++;
STATS->dump("all");
std::cout<<"currentIterationIns"<<currentIterationIns<<"\n";
    int r = rand()%5-2;
    switch (m.getmtype()) {
        case Move::Ins:
	    currentIterationIns++;
            TabuList[m] = currentIterationIns + tabuLengthIns + r;
            STATS->inc("tabu Ins Moves Added");
            break;
        case Move::IntraSw:
	    currentIterationIntraSw++;
            TabuList[m] = currentIterationIntraSw + tabuLengthIntraSw + r;
            STATS->inc("tabu IntraSw Moves Added");
            break;
        case Move::InterSw:
	    currentIterationInterSw++;
            TabuList[m] = currentIterationInterSw + tabuLengthInterSw + r;
            STATS->inc("tabu InterSw Moves Added");
            break;
    }
    //cleanExpired();  
    addToStats(m);
#ifndef LOG
dumpTabuList();
#endif
}


void savingsStats(const Move &move) const{
    if (move.getsavings() < 0) {
        STATS->inc("neg savings applied");
        switch  (move.getmtype()) {
                case Move::Ins: STATS->inc("neg sav Ins applied");break;
                case Move::IntraSw: STATS->inc("neg sav IntraSw applied"); break;
                case Move::InterSw: STATS->inc("neg sav InterSw applied"); break;
        }
    } else {
        STATS->inc("pos savings applied");
        switch  (move.getmtype()) {
                case Move::Ins: STATS->inc("pos sav Ins applied");break;
                case Move::IntraSw: STATS->inc("pos sav IntraSw applied"); break;
                case Move::InterSw: STATS->inc("pos sav InterSw applied"); break;
        }
    }
};
                                            


void addToStats(const Move &move) const {
         switch ( move.getmtype()) {
                    case Move::Ins:     STATS->inc("cnt Ins Applied");    break;
                    case Move::IntraSw: STATS->inc("cnt IntraSw Applied"); break;
                    case Move::InterSw: STATS->inc("cnt InterSw Applied"); break;
         }
        savingsStats(move);
};


void removeTruckFromTabuList(POS truckPos) {
#ifndef TESTED
std::cout<<"Entering TabuBase::removeTruckFromTabuList \n";
#endif
        int pos1,pos2;
        Move move;
        int expires;
        std::map<const Move, int> oldTabuList=TabuList;
dumpTabuList();
        TabuList.clear();

        for (std::map<Move,int>::iterator it = oldTabuList.begin(); it!=oldTabuList.end(); ++it) {
            pos1= it->first.getvid1();
            pos2= it->first.getvid2();
            move=it->first;
            expires= it->second;
            if ( pos1 == truckPos or pos2 == truckPos ) continue;
            if  ( pos1 > truckPos)  move.setvid1( pos1-1 ); //interface for position is with vid
            if  ( pos2 > truckPos)  move.setvid2( pos2-1 ); //interface for position is with vid
            TabuList[move]=expires;
        }
dumpTabuList();
}





};

#endif

