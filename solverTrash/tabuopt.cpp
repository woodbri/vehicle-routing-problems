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

#ifdef VICKY
/** 
Before doing any optimization within the trucks, the truck number must be reduced as much as possible



*/
void optimizeTruckNumber(){
	std::deque<int> Z1_fullTrucks; /**< trucks with last trip completly full */
	std::deque<int> Z2_fullTrucks; /**< trucks with extra trip completly full or cant be done */
	

}

/**
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
#ifndef TESTED
std::cout<<"Entering TabuOpt::search() \n";
#endif

    std::deque<Move> aspirationalTabu;
    std::deque<Move> nonTabu;
    std::deque<Move> tabu;
    currentIteration = 0;
    maxIteration = 1;

    Timer start;
    bool improvedBest;
    int lastImproved = 0;
    bestSolution.optimizeTruckNumber();

    do {
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration
            << std::endl;

        // this is a token ring search
        improvedBest  = doNeighborhoodMoves(Ins,     limitIns*50, aspirationalTabu, nonTabu,tabu);
        //improvedBest |= doNeighborhoodMoves(InterSw, limitInterSw*5, aspirationalTabu, nonTabu, tabu);
        //improvedBest |= doNeighborhoodMoves(IntraSw, limitIntraSw*100, aspirationalTabu, nonTabu,tabu);

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





void TabuOpt::getNeighborhood( neighborMovesName whichNeighborhood, std::deque<Move> &neighborhood,double factor) const {
        neighborhood.clear();
#ifdef TESTED
std::cout<<"Entering TabuOpt::getNeighborhod() \n";
#endif
        Timer getNeighborhoodTimer;
        switch (whichNeighborhood) {
            case Ins:
                ++currentIterationIns;
                currentSolution.v_getInsNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("Ins", getNeighborhoodTimer.duration(), neighborhood.size());
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
std::cout<<"Exiting TabuOpt::getNeighborhod() \n";
#endif
}


bool TabuOpt::applyAspirationalTabu(std::deque<Move> & aspirationalTabu) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::applyAspirationalTabu() \n";
#endif
	assert (aspirationalTabu.size());
        std::sort(aspirationalTabu.begin(), aspirationalTabu.end(), Move::bySavings); //the list is short so not so yikes (1 per local neighborhood)

        STATS->set("best Updated Last At", currentIteration);
        STATS->inc("best Updated Cnt");

        currentSolution.v_applyMove(aspirationalTabu[0]);  //allways the best even if negative
        makeTabu(aspirationalTabu[0]);
        bestSolution = currentSolution;
        computeCosts(bestSolution);
        bestSolutionCost = bestSolution.getCost();
	addToStats(aspirationalTabu[0]);
#ifndef TESTED
std::cout<<"Exiting TabuOpt::applyAspirationalTabu made \n"; aspirationalTabu[0].dump();
#endif
        return true;
}

#ifdef TOBEDELETED
void TabuOpt::addToStats(const Move &move) const {
	 switch ( move.getmtype()) {
                    case Move::Ins:     STATS->inc("cnt Ins Applied");    break;
                    case Move::IntraSw: STATS->inc("cnt IntraSw Applied"); break;
                    case Move::InterSw: STATS->inc("cnt InterSw Applied"); break;
         }
	savingsStats(move);
};
#endif

bool TabuOpt::applyNonTabu (std::deque<Move> &notTabu) {
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

bool  TabuOpt::applyTabu (std::deque<Move> &tabu) {
        assert ( tabu.size() );   //cant apply, there are non saved
	return applyTabu(tabu,0);
}

bool TabuOpt::applyTabu (std::deque<Move> &tabu, int strategy) {
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

bool TabuOpt::doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxMoves, 
    std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu, std::deque<Move> &tabu) { 

#ifndef TESTED
std::cout<<"Entering TabuOpt::doNeighobrhoodMoves\n";
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

	getNeighborhood(whichNeighborhood,neighborhood,factor); 
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


        if ( applyAspirationalNotTabu( neighborhood, aspirationalTabu, notTabu, tabu) ) {  //improved best & its not tabu
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
                applyAspirationalTabu( aspirationalTabu );
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
		applyNonTabu( notTabu );
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
		applyTabu( tabu,0 ); //random

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
std::cout<<"Exiting TabuOpt::doNeighobrhoodMoves\n";
#endif
    return improvedBest;
}

/**
	
*/
bool TabuOpt::applyAspirationalNotTabu (std::deque<Move> &neighborhood, std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu,std::deque<Move> &tabu) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::applyAspirationalNotTabu() \n";
#endif
        Timer applyAspirationalNotTabuTimer;
//        bool allTabu = true;
	bool found = false;
	bool foundAspTabu = false;
	double newCost;
	computeCosts(currentSolution);
        double actualCost= currentSolution.getCost();
        OptSol current=currentSolution;

        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {
	    current=currentSolution;
            current.v_applyMove(*it);
	    computeCosts(current);
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
                computeCosts(bestSolution);
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
std::cout<<"Exiting TabuOpt::applyAspirationalNotTabu  move made \n";
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
	
void TabuOpt::computeCosts(OptSol &s) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::computeCosts \n";
#endif
        int removedTruck = s.v_computeCosts();
        if (removedTruck==-1) return;
	removeTruckFromTabuList(removedTruck) ;
	return;
}





#endif
