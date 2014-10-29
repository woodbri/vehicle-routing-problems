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

#include <iostream>
#include <deque>
#include <algorithm>
#include <cstdlib>

#include "trashstats.h"
#include "optsol.h"
#include "tabuopt.h"

#ifdef TESTED
void TabuOpt::dumpTabuList() const {
    std::map<const Move, int>::const_iterator it;

    std::cout << "TabuList at iteration: " << currentIteration << std::endl;
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
        it->first.dump();
        std::cout << " - expires: " << it->second << std::endl;
    }
    std::cout << "--------------------------" << std::endl;
}


void TabuOpt::dumpStats() const {
    std::cout << "TabuList Stats at iteration: " << currentIteration << std::endl;
    STATS->dump("");
}


void TabuOpt::generateNeighborhoodStats(std::string mtype, double tm, int cnt) const {
    STATS->addto("time Gen "+mtype, tm);
    STATS->inc("cnt Calls Gen "+mtype);
    STATS->addto("cum Moves "+mtype, cnt);
std::cout << "\tdoNeighborhoodMoves for " << mtype << ": " << cnt
<< " moves generated" << std::endl;
}



bool TabuOpt::isTabu(const Move& m) const {
    std::map<const Move, int>::const_iterator it;

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
        if (m.isForbidden(it->first)) {
            STATS->inc("tabu Moves Checked Tabu");
            return true;
        }
    }
    return false;
}


void TabuOpt::cleanExpired() {
    std::map<const Move, int>::iterator it;
    for (it = TabuList.begin(); it!=TabuList.end(); ++it)
        if (it->second < currentIteration)
            TabuList.erase(it);
}


void TabuOpt::makeTabu(const Move &m) {
#ifdef VICKY
std::cout<<"makeTabu\n";
m.dump();
std::cout<<"endMove\n";
#endif
    // generate a randon value between -2 and +2
    // to adjust the tabu length with
    int r = rand()%5-2;
    switch (m.getmtype()) {
        case Move::Ins:
            TabuList[m] = currentIterationIns + tabuLengthIns + r;
            STATS->inc("tabu Ins Moves Added");
            break;
        case Move::IntraSw:
            TabuList[m] = currentIterationIntraSw + tabuLengthIntraSw + r;
            STATS->inc("tabu IntraSw Moves Added");
            break;
        case Move::InterSw:
            TabuList[m] = currentIterationInterSw + tabuLengthInterSw + r;
            STATS->inc("tabu InterSw Moves Added");
#ifndef VICKY
assert(true==false);
#endif
            break;
    }
}


/*
    This Tabu search algorithm was adapted from the paper:

    "Tabu Search Techniques for the Hetrogeneous Vehicle Routing
    Problem with Time Windows and Carrier-Dependent Cots" by
    Sara Ceschia, Luca Di Gaspero, and Andrea Schaerf

    We use a sequential solving strategy for combining our three neighborhood
    move functions. This as know as a "token-ring" search. Given an initial
    state and a set of algorithms, it makes circularly a run at each algorithm,
    always starting from the best solution found by the previous one. The
    overall process stops either when a full round of the algorithms does not
    find an improvement or the time (aka: interation count) granted has elapsed.
    
    Each single algorithm stops when it does not improve the current best
    solution for a given number of iterations (ie: stagnation).

*/
void TabuOpt::search() {

    currentIteration = 0;

    Timer start;
    bool improvedBest;
    int lastImproved = 0;
    do {
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration
            << std::endl;
        // [article]
        // set the stagnation count as the last the last parameter
        // the values 500, 300, 300 can from the paper mentioned above
        //
        // this is a token ring search
        // for each iteration we process all moves available
        // from each slot (ie: Ins, IntraSw, InterSw)
        // when there are no move moves we terminate
        // or if we reach stagnation. Stagnation is N moves
        // without improving the bestSolution
        improvedBest  = doNeighborhoodMoves(Ins,     500);
        improvedBest |= doNeighborhoodMoves(IntraSw, 300);
        improvedBest |= doNeighborhoodMoves(InterSw, 300);

        if (improvedBest) lastImproved = 0;
        else ++lastImproved;

        std::cout << "TABUSEARCH: Finished iteration: " << currentIteration
            << ", improvedBest: " << improvedBest
            << ", run time: " << start.duration()
            << std::endl;

        STATS->set("0 Iteration", currentIteration);
        STATS->set("0 Best Cost After", bestSolution.getCost());
        dumpStats();
        std::cout << "--------------------------------------------\n";
    }
    //while (improvedBest and ++currentIteration < maxIteration);
    while (lastImproved < 1 and ++currentIteration < maxIteration);

    std::cout << "TABUSEARCH: Total time: " << start.duration() << std::endl;
}

void TabuOpt::generateNeighborhood(neighborMovesName whichNeighborhood, std::deque<Move>& neighborhood, const Move& lastMove) const {

    // generate the a move neighborhood based on the currentSolution
    // this is fast and efficient as move generation accounts for most
    // of the computational time.
    // We only look at a very small percentage of the actual moves
    // but we have to calcuate savings and feasiblity on all of them

std::cout << "generateNeighborhood: lastMove: "; lastMove.dump();

    Timer timeNeighboorhoodGeneration;
    switch (whichNeighborhood) {
        case Ins:
            ++currentIterationIns;
            //currentSolution.v_getInsNeighborhood(neighborhood, factor);
            currentSolution.getInsNeighborhood(neighborhood, lastMove);

            // collect stats
            generateNeighborhoodStats("Ins",
                    timeNeighboorhoodGeneration.duration(),
                    neighborhood.size());
            break;
        case IntraSw:
            ++currentIterationIntraSw;
            //currentSolution.v_getIntraSwNeighborhood(neighborhood, factor);
            currentSolution.getIntraSwNeighborhood(neighborhood, lastMove);

            // collect stats
            generateNeighborhoodStats("IntraSw",
                    timeNeighboorhoodGeneration.duration(),
                    neighborhood.size());
            break;
        case InterSw:
            ++currentIterationInterSw;
            //currentSolution.v_getInterSwNeighborhood(neighborhood, factor);
            currentSolution.getInterSwNeighborhood(neighborhood, lastMove);

            // collect stats
            generateNeighborhoodStats("InterSw",
                    timeNeighboorhoodGeneration.duration(),
                    neighborhood.size());
            break;
    }
}


/*
    Algorithm for processing neighborhood moves [article]

    For the requested individual move neighborhood:
        doInsMoves, doIntraSwMoves, doInterSwMoves

    do {
        Generate the neighborhood of moves and order from best to worst.
        Working through the neighborhood (best to worst)
        Filter out inFeasible moves then
        if the move is aspirational we apply the move
        otherwise if the move is not Tabu apply the move
        even if it makes the current solution worse.
        If all moves are tabu, then apply the best one
    } until there are not valid moves or stagnation 
    return an indicator that we improved the best move or not.
*/

bool TabuOpt::doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxStagnation) {
    bool improvedBest = false;
    int stagnationCnt = 0;
    double factor = 0.5;
    std::string mName;

    switch (whichNeighborhood) {
        case Ins:     mName = "Ins";    break;
        case IntraSw: mName = "IntraSw"; break;
        case InterSw: mName = "InterSw"; break;
    }

    // we always start from the best solution of the last run [article]
    currentSolution = bestSolution;

    STATS->set("factor", factor);

    std::deque<Move> neighborhood;
    Move lastMove;

    do {
        // generate or regenerate the neighborhood moves
        generateNeighborhood(whichNeighborhood, neighborhood, lastMove);

        // and sort it so we can work from the best to the worst
        std::sort(neighborhood.begin(), neighborhood.end(), Move::bySavings);

// dump the neighborhood
//        for (int i=0;i<neighborhood.size();i++) neighborhood[i].dump();
//        std::cout<<"======================================================";

        // take the best move that we may apply and apply it, if any
        Timer applyMoveTimer;
        bool allTabu = true;
        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {

            // if the move is aspirational then we apply it
            if (currentSolution.getCost() - it->getsavings() < bestSolutionCost) {
std::cout << "\tdoNeighborhoodMoves: Aspiration move: "; it->dump();
                currentSolution.applyMove(*it);
                makeTabu(*it);
                lastMove = *it;

                bestSolution = currentSolution;
                bestSolutionCost = bestSolution.getCost();
                improvedBest = true;
                stagnationCnt = 0;
                allTabu = false;
                STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
                STATS->inc("cnt Applied " + mName);

                // ok we made a move, so now the neighborhood is no
                // longer valid so break to regenerate a new neighborhood
                break;
            }
            // if the move is not Tabu, then we apply it even if
            // it makes the solution worse so we move to a new
            // area of the search space
            else if (! isTabu(*it)) {
std::cout << "\tdoNeighborhoodMoves: Not Tabu: "; it->dump();
                currentSolution.applyMove(*it);
                makeTabu(*it);
                lastMove = *it;

                allTabu = false;
                STATS->inc("cnt Applied " + mName);

                // ok we made a move, so now the neighborhood is no
                // longer valid so break to regenerate a new neighborhood
                break;
            }
            else {
                ++stagnationCnt;
            }
        }

        // if all the moves are tabu then we need to pick one
        // the articles says to pick the best
        if (allTabu and neighborhood.size()) {
            int pick = 0; // pick the best

            lastMove = *(neighborhood.begin()+pick);
            currentSolution.applyMove(lastMove);
            makeTabu(lastMove);

std::cout << "\tdoNeighborhoodMoves: All Tabu(" << pick << "): "; lastMove.dump();
            STATS->inc("cnt Applied " + mName);
        }
        STATS->addto("time Apply Moves", applyMoveTimer.duration());
    }
    while (stagnationCnt < maxStagnation );

    if (not improvedBest)
        std::cout << "\tStagnation reached in neighborhood: " << whichNeighborhood << std::endl;

    return improvedBest;
}
#endif

///////////////////////////////////////////////////
#ifdef VICKY


void TabuOpt::v_search() {
#ifndef TESTED
std::cout<<"Entering TabuOpt::v_search() \n";
#endif

    std::deque<Move> aspirationalTabu;
    std::deque<Move> nonTabu;
    std::deque<Move> tabu;
    currentIteration = 0;
    maxIteration = 1;

    Timer start;
    bool improvedBest;
    int lastImproved = 0;
    do {
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration
            << std::endl;

        // this is a token ring search
        improvedBest  = v_doNeighborhoodMoves(Ins,     limitIns*50, aspirationalTabu, nonTabu,tabu);
        //improvedBest |= v_doNeighborhoodMoves(InterSw, limitInterSw*5, aspirationalTabu, nonTabu, tabu);
        //improvedBest |= v_doNeighborhoodMoves(IntraSw, limitIntraSw*100, aspirationalTabu, nonTabu,tabu);

        if (improvedBest) lastImproved = 0;
        else ++lastImproved;

        std::cout << "TABUSEARCH: Finished iteration: " << currentIteration
            << ", improvedBest: " << improvedBest
            << ", run time: " << start.duration()
            << std::endl;

        STATS->set("0 Iteration", currentIteration);
        STATS->set("0 Best Cost After", bestSolution.getCost());
        dumpStats();
        std::cout << "--------------------------------------------\n";
    }
    //while (improvedBest and ++currentIteration < maxIteration);
    while (lastImproved < 1 and ++currentIteration < maxIteration);

    std::cout << "TABUSEARCH: Total time: " << start.duration() << std::endl;
}





void TabuOpt::v_getNeighborhood( neighborMovesName whichNeighborhood, std::deque<Move> &neighborhood,double factor) const {
        neighborhood.clear();
#ifdef TESTED
std::cout<<"Entering TabuOpt::v_getNeighborhod() \n";
#endif
        Timer v_getNeighborhoodTimer;
        switch (whichNeighborhood) {
            case Ins:
                ++currentIterationIns;
                currentSolution.v_getInsNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("v_Ins", v_getNeighborhoodTimer.duration(), neighborhood.size());
//std::cout << "\tdoNeighborhoodMoves for Ins: " << neighborhood.size() << " moves generated" << std::endl;

                break;
            case IntraSw:
                ++currentIterationIntraSw;
                currentSolution.v_getIntraSwNeighborhood(neighborhood, factor);

//std::cout << "\tdoNeighborhoodMoves for IntraSw: " << neighborhood.size() << " moves generated" << std::endl;
                break;
            case InterSw:
                ++currentIterationInterSw;
                currentSolution.v_getInterSwNeighborhood(neighborhood, factor);

//std::cout << "\tdoNeighborhoodMoves for InterSw: " << neighborhood.size() << " moves generated" << std::endl;
                break;
        }
#ifdef TESTED
std::cout<<"Exiting TabuOpt::v_getNeighborhod() \n";
#endif
}


bool TabuOpt::v_applyAspirationalTabu(std::deque<Move> & aspirationalTabu) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::v_applyAspirationalTabu() \n";
#endif
	assert (aspirationalTabu.size());
        std::sort(aspirationalTabu.begin(), aspirationalTabu.end(), Move::bySavings); //the list is short so not so yikes (1 per local neighborhood)

        STATS->set("best Updated Last At", currentIteration);
        STATS->inc("best Updated Cnt");

        currentSolution.v_applyMove(aspirationalTabu[0]);  //allways the best even if negative
        makeTabu(aspirationalTabu[0]);
        bestSolution = currentSolution;
        v_computeCosts(bestSolution);
        bestSolutionCost = bestSolution.getCost();
	addToStats(aspirationalTabu[0]);
#ifndef TESTED
std::cout<<"Exiting TabuOpt::v_applyAspirationalTabu made \n"; aspirationalTabu[0].dump();
#endif
        return true;
}

#ifdef TOBEDELETED
void TabuOpt::v_addToStats(const Move &move) const {
	 switch ( move.getmtype()) {
                    case Move::Ins:     STATS->inc("cnt Ins Applied");    break;
                    case Move::IntraSw: STATS->inc("cnt IntraSw Applied"); break;
                    case Move::InterSw: STATS->inc("cnt InterSw Applied"); break;
         }
	v_savingsStats(move);
};
#endif

#ifdef NOTWORKING
bool TabuOpt::v_applyAspirational(std::deque<Move> &neighborhood, std::deque<Move> &notTabu,std::deque<Move> &tabu) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::v_applyAspirational() \n";
#endif
        Timer applyMoveTimer;
        bool allTabu = true;
	bool found = false;
	currentSolution.computeCosts();
        double actualCost= currentSolution.getCost();
        Neighborhoods current=currentSolution;
        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {
	    current=currentSolution;
            current.v_applyMove(*it);
	    current.computeCosts();
	    double newCost = current.getCost();
	    if (not ( (std::abs( (actualCost - newCost)  -  it->getsavings())) <0.5) ) {
		it->dump();
		std::cout<<"something is wrong with the savings****** "<<it->getsavings()<< " ***** "<<actualCost - newCost<<"\n";
            } 
            // if the move is aspirational then we apply it
            if (current.getcost()  < bestSolutionCost  ) {  
std::cout << "\tapplyAspirational: Aspiration move: "; it->dump();
                currentSolution=current;
                bestSolution = current;
                v_computeCosts(bestSolution);
                bestSolutionCost = bestSolution.getCost();
                makeTabu(*it);
                STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
		v_addToStats(*it); // update stats

                   neighborhood.clear();
	           notTabu.clear();
	           tabu.clear();
//assert(true==false);
#ifndef TESTED
std::cout<<"Exiting TabuOpt::v_applyAspirational  move made \n";
#endif
	           return true;
		
            }
            if (! isTabu(*it) and not found ) { 			//save only the best nonTabu move
		notTabu.push_back(*it); 
		found = true;
std::cout << "\tapplyApsirational: nonTabu Move found: "; it->dump();
		tabu.clear();  //no need to keep the tabus
	    } else if ( not notTabu.size() ) { 
		tabu.push_back(*it); //we dont have a nonTabu move from other local neighborhood, so we have to save all moves
            }	
        };
#ifdef TESTED
std::cout << "\napplyApsirational: we didnt make  aspirational move \t "; 
std::cout<<" notTabu size "<<notTabu.size()<<" \t";
std::cout<<" Tabu size "<< tabu.size()<<" \n";
std::cout << "\tapplyApsirational: exiting "; 
#endif

	return false;          
};
#endif



bool TabuOpt::v_applyNonTabu (std::deque<Move> &notTabu) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyNonTabu() \n";
#endif
        assert (  notTabu.size() ) ;  //cant apply, there are non saved
        std::sort(notTabu.begin(), notTabu.end(), Move::bySavings); //the list is short so not so yikes (1 per local neighborhood)
std::cout << "\tapplyNonTabu: Not Tabu: "; notTabu[0].dump();
std::cout << "\n";

        currentSolution.v_applyMove(notTabu[0]);  //allways the best even if negative
        makeTabu(notTabu[0]);
	addToStats(notTabu[0]);
	return true;
}

bool  TabuOpt::v_applyTabu (std::deque<Move> &tabu) {
        assert ( tabu.size() );   //cant apply, there are non saved
	return v_applyTabu(tabu,0);
}

bool TabuOpt::v_applyTabu (std::deque<Move> &tabu, int strategy) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyTabu #of possible moves:"<<tabu.size()<<"\n";
#endif
        assert( tabu.size() );  //cant apply, there are non saved
	
        if (strategy==0) {  //pick Best
            currentSolution.v_applyMove(tabu[0]);
            makeTabu(tabu[0]);
std::cout << "\tapplyTabu:  best: "; tabu[0].dump();
	    addToStats(tabu[0]);
	} else {
          int pickWorse = rand()% ( tabu.size()-1 );
std::cout << "\tapplyTabu: pickworse"<<pickWorse<<"\n"; tabu[pickWorse].dump();
            currentSolution.v_applyMove(tabu[pickWorse]);
            makeTabu(tabu[pickWorse]);
	    addToStats(tabu[pickWorse]);
	}
	tabu.clear();
#ifndef TESTED
std::cout<<"Exiting TabuOpt::applyTabu #of possible moves:"<<tabu.size()<<"\n";
#endif
	return true;
}

bool TabuOpt::reachedMaxCycles(int number, neighborMovesName whichNeighborhood) {
	bool limit;
        switch (whichNeighborhood) {
               case Ins: { limit = number>limitIns; }
               case IntraSw: { limit = number>limitIntraSw; }
               case InterSw: { limit = number>limitInterSw; }
        };
	return limit;
}


bool TabuOpt::v_doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxMoves, 
    std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu, std::deque<Move> &tabu) { 

#ifndef TESTED
std::cout<<"Entering TabuOpt::v_doNeighobrhoodMoves\n";
#endif
    bool improvedBest = false;
    int Cnt = 0;
    int CntNonAspirational =0;
    int CntNoNeighborhood =0;
    double factor = 0.5;
    bool limit;

    // we always start from the best solution of the last run
    //currentSolution = bestSolution;

    STATS->set("factor", factor);
    std::deque<Move> neighborhood;

    do {
        std::string solBefore = currentSolution.solutionAsText();

	v_getNeighborhood(whichNeighborhood,neighborhood,factor); 
	if (not neighborhood.size()) { 
		CntNoNeighborhood++; 
std::cout<<" No Moves are found  "<< CntNoNeighborhood <<"\n";
		if ( reachedMaxCycles(CntNoNeighborhood,whichNeighborhood) ) {
std::cout<<" Reached end of cycle -No moves found- "<<Cnt<<" out of "<< maxMoves<<"\n";
		   return improvedBest; //we cycled and no neighborhood moves were found
                } else continue; 
        };

	CntNoNeighborhood=0; 
        std::sort(neighborhood.begin(), neighborhood.end(), Move::bySavings); //yikes

        std::string solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);


        if ( v_applyAspirationalNotTabu( neighborhood, aspirationalTabu, notTabu, tabu) ) {  //improved best & its not tabu
                Cnt++;
		CntNonAspirational=0;
		improvedBest = true;
		neighborhood.clear();
		aspirationalTabu.clear();
		notTabu.clear();
		tabu.clear();
                continue;
	}

//std::cout<<" aspirationalTabu size "<<aspirationalTabu.size()<<" \n";
//std::cout<<" notTabu size "<<notTabu.size()<<" \n";
//std::cout<<" Tabu size "<< tabu.size()<<" \n";

	if ( reachedMaxCycles(aspirationalTabu.size(),whichNeighborhood) ) {
                Cnt++;
                v_applyAspirationalTabu( aspirationalTabu );
                CntNonAspirational=0;
                neighborhood.clear();
                aspirationalTabu.clear();
                tabu.clear();
                notTabu.clear();
                continue;
        }

//std::cout<<" notTabu size "<<notTabu.size()<<" \n";
//std::cout<<" Tabu size "<< tabu.size()<<" \n";

	if ( reachedMaxCycles(notTabu.size(),whichNeighborhood) ) {
                Cnt++;
		v_applyNonTabu( notTabu );
		CntNonAspirational=0;
		neighborhood.clear();
                aspirationalTabu.clear();
		tabu.clear();	
		notTabu.clear();	
		continue;
	} 
	CntNonAspirational++;
	if (notTabu.size()) continue;
	
        switch (whichNeighborhood) {
                 case Ins: { limit = CntNonAspirational>limitIns; }
                 case IntraSw: { limit = CntNonAspirational>limitIntraSw; }
                 case InterSw: { limit = CntNonAspirational>limitInterSw; }
        };
        solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);
	if ( tabu.size()>0 and reachedMaxCycles(CntNonAspirational ,whichNeighborhood) ) {
	//if ( limit ) {  // we cycled thru all the local neighborhoods and no non aspirational nor non Tabu move was found 
                Cnt++;
		v_applyTabu( tabu,0 ); //random

		CntNonAspirational=0;
		neighborhood.clear();
                aspirationalTabu.clear();
		tabu.clear();
		notTabu.clear();	
	        break;
       }  
    }	
    while ( Cnt < maxMoves );

#ifndef TESTED
std::cout<<" Moves made "<<Cnt<<" out of "<< maxMoves<<"\n";
std::cout<<"Exiting TabuOpt::v_doNeighobrhoodMoves\n";
#endif
    return improvedBest;
}

/**
	
*/
bool TabuOpt::v_applyAspirationalNotTabu (std::deque<Move> &neighborhood, std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu,std::deque<Move> &tabu) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::v_applyAspirationalNotTabu() \n";
#endif
        Timer applyAspirationalNotTabuTimer;
//        bool allTabu = true;
	bool found = false;
	bool foundAspTabu = false;
	double newCost;
	v_computeCosts(currentSolution);
        double actualCost= currentSolution.getCost();
        OptSol current=currentSolution;

        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {
	    current=currentSolution;
            current.v_applyMove(*it);
	    v_computeCosts(current);
	    newCost = current.getCost();

	    if (not ( (std::abs( (actualCost - newCost)  -  it->getsavings())) <0.5) ) {
		it->dump();
		std::cout<<"something is wrong with the savings****** "<<it->getsavings()<< " ***** "<<actualCost - newCost<<"\n";
            } 
            // if the move is aspirational then we apply it

            if (current.getcost()  < bestSolutionCost and not isTabu(*it) ) {  
std::cout << "\tapplyAspirationalNotTabu:  "; it->dump();
                currentSolution=current;
                bestSolution = current;
                v_computeCosts(bestSolution);
                bestSolutionCost = bestSolution.getCost();
                makeTabu(*it);
                STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
		addToStats(*it); // update stats

                // ok we made a move, so now the neighborhood is no longer valid we discard the tabu and the nonTabu found and any other aspirational tabu 
                   neighborhood.clear();
		   aspirationalTabu.clear();
	           notTabu.clear();
	           tabu.clear();
//assert(true==false);
#ifndef TESTED
std::cout<<"Exiting TabuOpt::v_applyAspirationalNotTabu  move made \n";
#endif
	           return true;
		
            }

	   if (newCost  < bestSolutionCost and not foundAspTabu) { //save only the best aspirational Tabu
		aspirationalTabu.push_back(*it);
		foundAspTabu = true;	
std::cout << "\tapplyApsirationalNotTabu: apsirationalTabu Move found: "; it->dump();
std::cout << "\tnewCost:"<<newCost<< "\tbestSolutionCost"<<bestSolutionCost<<"\n "; 
std::cout<<"\n";

		tabu.clear();  //no need to keep the tabus
	   } else if (! isTabu(*it) and not found ) { 			//save only the best nonTabu move
		notTabu.push_back(*it); 
		found = true;
//std::cout << "\tapplyApsirational: nonTabu Move found: "; it->dump();
		tabu.clear();  //no need to keep the tabus
	    } else if ( not notTabu.size() ) { 
		tabu.push_back(*it); //we dont have a nonTabu move from other local neighborhood, so we have to save all moves
            }	
        };

	return false;          
};
#ifdef TOBEDELETED
void TabuOpt::v_savingsStats(const Move &move) const{
    move.dump();
    if (move.getsavings() < 0) {
        STATS->inc("neg savings applied");
        switch  (move.getmtype()) {
                case Move::Ins: STATS->inc("neg sav v_Ins applied");break;
                case Move::IntraSw: STATS->inc("neg sav v_IntraSw applied"); break;
                case Move::InterSw: STATS->inc("neg sav v_InterSw applied"); break;
        }
    } else {
        STATS->inc("pos savings applied");
        switch  (move.getmtype()) {
                case Move::Ins: STATS->inc("pos sav v_Ins applied");break;
                case Move::IntraSw: STATS->inc("pos sav v_IntraSw applied"); break;
                case Move::InterSw: STATS->inc("pos sav v_InterSw applied"); break;
        }
    }
};
#endif
	
void TabuOpt::v_computeCosts(OptSol &s) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::v_computeCosts \n";
#endif
        int removedTruck = s.v_computeCosts();
        if (removedTruck==-1) return;
	removeTruckFromTabuList(removedTruck) ;
	return;
	int vid1,vid2;
        Move move;
	int expires;
        std::map<Move,int>::iterator it = TabuList.begin();
        while (it != TabuList.end()) {
	    vid1= it->first.getvid1();
	    vid2= it->first.getvid2();
	    move=it->first;
	    expires= it->second;
move.dump();
            if ( vid1 == removedTruck or vid2 == removedTruck ) {
                TabuList.erase( it ); 
		it=TabuList.begin();
            } else {
		if  ( vid1 > removedTruck)  move.setvid1( vid1-1 );
            	if  ( vid2 > removedTruck)  move.setvid1( vid2-1 );
		if ( vid1 > removedTruck or  vid2 > removedTruck) {
		  TabuList.erase( it );
		  TabuList[move]=expires;
                }
	    }

        }
}


#endif

