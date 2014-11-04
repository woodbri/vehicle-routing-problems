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
    //maxIteration = 1;     // use ts.setMaxIteration(1) in trash.cpp

    Timer start;
    bool improvedBest;
    int lastImproved = 0;

    do {
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration << std::endl;

        // this is a token ring search
        improvedBest  = doNeighborhoodMoves(Ins, limitIns*10, aspirationalTabu, nonTabu,tabu);
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
                currentSolution.v_getInsNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("Ins", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
            case IntraSw:
                currentSolution.v_getIntraSwNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("IntraSw", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
            case InterSw:
                currentSolution.v_getInterSwNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("InterSw", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
        }
#ifdef TESTED
std::cout<<"Exiting TabuOpt::getNeighborhod() \n";
#endif
}

bool TabuOpt::applyAmove(const Move &move) {
#ifndef TESTED
std::cout<<"\nApply a move ";move.Dump();
#endif

        currentSolution.v_applyMove(move); 
        makeTabu(move);
        computeCosts(currentSolution);
        currentCost=currentSolution.getCost();
        if (bestSolutionCost > currentCost) {
		setCurrentAsBest();
        	STATS->set("best Updated Last At", currentIteration);
        	STATS->inc("best Updated Cnt");
	}
        return true;
}



bool TabuOpt::applyAspirationalNotTabu(const Move  &move) {
	applyAmove(move); 			//has only one move
        STATS->inc("cnt Aspirational Not Tabu");
#ifndef TESTED
std::cout<<"\nAspirational non Tabu aplied ";move.Dump();
#endif
	return true;
}

bool TabuOpt::applyAspirationalTabu(std::deque<Move> &moves) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyAspirationalTabu \n";
#endif
	assert (moves.size());
	if (not moves.size()) return false;
        std::sort(moves.begin(), moves.end(), Move::bySavings); 
#ifndef TESTED
std::cout<<"\nAspirational  Tabu aplied ";moves[0].Dump();
#endif
	applyAmove(moves[0]);
        STATS->inc("cnt Aspirational Tabu");
	moves.clear();
        return true;
}


bool TabuOpt::applyNonTabu (std::deque<Move> &moves) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyNonTabu() \n";
#endif
        assert (  moves.size() ) ;  //cant apply, there are non saved
	if (not moves.size()) return false;
        std::sort(moves.begin(), moves.end(), Move::bySavings); //the list is short so not so yikes (1 per local neighborhood)
#ifndef TESTED
std::cout<<"\nNon  Tabu aplied ";moves[0].Dump();
#endif
        applyAmove(moves[0]);
        STATS->inc("cnt Non Tabu");
        moves.clear();
	return true;
}

bool  TabuOpt::applyTabu (std::deque<Move> &tabu) {
        assert ( tabu.size() );   //cant apply, there are non saved
	return applyTabu(tabu,0);
}

bool TabuOpt::applyTabu (std::deque<Move> &moves, int strategy) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyTabu #of possible moves:"<<moves.size()<<"\n";
#endif
        assert( moves.size() );  //cant apply, there are non saved
	if (not moves.size()) return false;
        std::sort(moves.begin(), moves.end(), Move::bySavings); 
	
        if (strategy==0) {  //pick Best
std::cout << "\tapplyTabu:  best: "; moves[0].dump();
           applyAmove(moves[0]);
           STATS->inc("cnt Tabu best");
	} else {
          int pickWorse = rand()% ( moves.size()-1 );
std::cout << "\tapplyTabu: pickworse"<<pickWorse<<"\n"; moves[pickWorse].dump();
           applyAmove(moves[pickWorse]);
           STATS->inc("cnt Tabu random");
	}
	moves.clear();
#ifndef TESTED
std::cout<<"Exiting TabuOpt::applyTabu #of possible moves:"<<moves.size()<<"\n";
#endif
	return true;
}

bool TabuOpt::reachedMaxCycles(int number, neighborMovesName whichNeighborhood) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::reachedMaxCycles "<<number<<"\n";
std::cout<<limitIns<<"\t"<<limitIntraSw<<"\t "<<limitInterSw<<"\n";
#endif
	bool limit;
        switch (whichNeighborhood) {
               case Ins: { limit = number>limitIns; break;}
               case IntraSw: { limit = number>limitIntraSw; break;}
               case InterSw: { limit = number>limitInterSw; break;}
        };
std::cout<<"Exit of TabuOpt::reachedMaxCycles"<<limit<<"\n";
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
    int actualMoveCount=getTotalMovesMade();

    // we always start from the best solution of the last run
    //currentSolution = bestSolution;

    STATS->set("factor", factor);
    std::deque<Move> neighborhood;

    do {
std::cout<<(getTotalMovesMade()-actualMoveCount)<<" > " <<maxMoves<<"***************************************************\n";
	if ((getTotalMovesMade()-actualMoveCount) > maxMoves) break;
        std::string solBefore = currentSolution.solutionAsText();

	getNeighborhood(whichNeighborhood,neighborhood,factor); 
        Cnt++;

	if (not neighborhood.size()) { 
		CntNoNeighborhood++; 
std::cout<<" No Moves are found  "<< CntNoNeighborhood <<"\n";
		if ( reachedMaxCycles(CntNoNeighborhood,whichNeighborhood) ) {
std::cout<<" Reached end of cycle -No moves found- "<<Cnt<<" out of "<< maxMoves<<"\n";
		   return improvedBest; //we cycled and no neighborhood moves were found
                } else continue; 
        };

	CntNoNeighborhood=0; 

        std::string solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);

	if ( classifyMoves(neighborhood, aspirationalTabu, notTabu, tabu) ) {
		assert (not neighborhood.size());
		assert (not aspirationalTabu.size());
		assert (not notTabu.size());
		assert (not tabu.size());
		Cnt=0; //a move was made
		continue;
        } else {
		if (aspirationalTabu.size() ) 	assert ( not notTabu.size() and not tabu.size() );
		if (notTabu.size() ) assert ( not aspirationalTabu.size() and not tabu.size() );
		if (tabu.size() ) assert ( not aspirationalTabu.size() and not notTabu.size() );
	}

#ifdef LOG
	dumpMoves("neighborhood",neighborhood);
	dumpMoves("aspirationalTabu",aspirationalTabu);
	dumpMoves("notTabu",notTabu);
	std::cout<<" Tabu size:  "<< tabu.size() <<"\n";
#endif
        if (not reachedMaxCycles(Cnt,whichNeighborhood)) continue;

	if ( aspirationalTabu.size()>0 and reachedMaxCycles(Cnt, whichNeighborhood) ) {
		assert (not neighborhood.size());
		assert (aspirationalTabu.size());
		assert (not notTabu.size());
		assert (not tabu.size());

                applyAspirationalTabu( aspirationalTabu );
		Cnt=0;

		assert (not aspirationalTabu.size());
                continue;
        }


	if ( notTabu.size()>0 and reachedMaxCycles(Cnt,whichNeighborhood) ) {
		assert (not neighborhood.size());
		assert (not aspirationalTabu.size());
		assert (notTabu.size());
		assert (not tabu.size());
		applyNonTabu( notTabu );
		Cnt=0;
		neighborhood.clear();
		continue;
	} 

	if (notTabu.size()) continue;
	
        solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);
	if ( tabu.size()>0 and reachedMaxCycles(Cnt ,whichNeighborhood) ) {
		applyTabu( tabu,0 ); //best

		neighborhood.clear();
                aspirationalTabu.clear();
		tabu.clear();
		notTabu.clear();	
		Cnt=0;
	        continue;
       }  
    }	
    while (  (getTotalMovesMade()-actualMoveCount) < maxMoves );

#ifndef TESTED
std::cout<<" Moves made "<<Cnt<<" out of "<< maxMoves<<"\n";
std::cout<<"Exiting TabuOpt::doNeighobrhoodMoves\n";
#endif
    return improvedBest;
}

void TabuOpt::compareCostWithOSRM(std::deque<Move> &neighborhood) {
    int cnt = 0;
    Vehicle truckFrom, truckFromAfter;
    Vehicle truckTo, truckToAfter;
    Vehicle truck1, truck1After;
    Vehicle truck2, truck2After;
    double c_truckFrom,c_truckTo,co_truckFrom,co_truckTo,c_truckFromAfter,c_truckToAfter,co_truckFromAfter,co_truckToAfter,t_save,o_save,c_truck1,co_truck1,c_truck1After,co_truck1After,c_truck2,co_truck2,c_truck2After,co_truck2After;

    for (std::deque<Move>::iterator it=neighborhood.begin();
            it!=neighborhood.end(); ++it) {
        std::cout << "---------- compareWithCostOSRM ------------\n";
        it->Dump();
        OptSol current=currentSolution;
        current.v_applyMove(*it);
        bool removedTruck = computeCosts(current);
        double newCost = current.getcost();
        std::cout << "currentSolution cost: " << currentSolution.getcost()
                  << " ( " << currentSolution.getCost() << " )"
                  << std::endl;
        std::cout << "Applied move Sol cost: " << newCost
                  << " ( " << current.getCost() << " )"
                  << std::endl;
        std::cout << "         \tbefore\t\tafter\t\tbefore\t\tafter\n";
        std::cout << "         \tcost  \t\tcost \t\tosrm  \t\tosrm\n";
        switch (it->getmtype()) {
            case Move::Ins:
                truckFrom = currentSolution[it->getInsFromTruck()];
                truckTo   = currentSolution[it->getInsToTruck()];
                truckFromAfter = current[it->getInsFromTruck()];
                truckToAfter   = current[it->getInsToTruck()];
                c_truckFrom       = truckFrom.getcost();
                c_truckTo         = truckTo.getcost();
                co_truckFrom      = truckFrom.getCostOSRM();
                co_truckTo        = truckTo.getCostOSRM();
                c_truckFromAfter  = truckFromAfter.getcost();
                c_truckToAfter    = truckToAfter.getcost();
                co_truckFromAfter = truckFromAfter.getCostOSRM();
                co_truckToAfter   = truckToAfter.getCostOSRM();
                std::cout << "truckFrom\t" << c_truckFrom
                          << "\t" << c_truckFromAfter
                          << "\t" << co_truckFrom
                          << "\t" << co_truckFromAfter << std::endl;
                std::cout << "  truckTo\t" << c_truckTo
                          << "\t" << c_truckToAfter
                          << "\t" << co_truckTo
                          << "\t" << co_truckToAfter << std::endl;
                t_save = (c_truckFrom + c_truckTo) -
                         (c_truckFromAfter + c_truckToAfter);
                o_save = (co_truckFrom + co_truckTo) -
                         (co_truckFromAfter + co_truckToAfter);
                std::cout << "truck sav: " << t_save
                          << ", osrm sav: " << o_save << std::endl;
                break;
            case Move::IntraSw:
                truck1      = currentSolution[it->getIntraSwTruck()];
                truck1After = current[it->getIntraSwTruck()];
                c_truck1       = truck1.getcost();
                co_truck1      = truck1.getCostOSRM();
                c_truck1After  = truck1After.getcost();
                co_truck1After = truck1After.getCostOSRM();
                std::cout << "truck1\t" << c_truck1
                          << "\t" << c_truck1After
                          << "\t" << co_truck1
                          << "\t" << co_truck1After << std::endl;
                t_save = c_truck1  - c_truck1After;
                o_save = co_truck1 - co_truck1After;
                std::cout << "truck sav: " << t_save
                          << ", osrm sav: " << o_save << std::endl;
                break;
            case Move::InterSw:
                truck1 = currentSolution[it->getInterSwTruck1()];
                truck2 = currentSolution[it->getInterSwTruck2()];
                truck1After = current[it->getInterSwTruck1()];
                truck2After = current[it->getInterSwTruck2()];
                c_truck1       = truck1.getcost();
                c_truck2       = truck2.getcost();
                co_truck1      = truck1.getCostOSRM();
                co_truck2      = truck2.getCostOSRM();
                c_truck1After  = truck1After.getcost();
                c_truck2After  = truck2After.getcost();
                co_truck1After = truck1After.getCostOSRM();
                co_truck2After = truck2After.getCostOSRM();
                std::cout << "truck1\t" << c_truck1
                          << "\t" << c_truck1After
                          << "\t" << co_truck1
                          << "\t" << co_truck1After << std::endl;
                std::cout << "  truck2\t" << c_truck2
                          << "\t" << c_truck2After
                          << "\t" << co_truck2
                          << "\t" << co_truck2After << std::endl;
                t_save = (c_truck1 + c_truck2) -
                         (c_truck1After + c_truck2After);
                o_save = (co_truck1 + co_truck2) -
                         (co_truck1After + co_truck2After);
                std::cout << "truck sav: " << t_save
                          << ", osrm sav: " << o_save << std::endl;
                break;
        }
        if (cnt++ > 5) break;
    }
    std::cout << "-------------------------------------------\n";
}

/**
  Classify the moves into:
	aspirational Not tabu	(if found the buckets bellow are cleared)
	aspirational tabu	(adds to the bucket the best move)
	not Tabu		(adds to the bucket the best move) 
	Tabu			(adds to the bucket all the moves)

  if during the classification a truck is removed:
	move is applied
	all buckets are cleared


	returns true: any kind of move was made
*/
bool TabuOpt::classifyMoves (std::deque<Move> &neighborhood,  std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu,std::deque<Move> &tabu) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::classifyMoves \n";
#endif
        Timer start;
	bool found = false;
	bool foundAspTabu = false;
	bool removedTruck = false;  
	double newCost;
	computeCosts(currentSolution);
        double actualCost= currentSolution.getCost();
        OptSol current=currentSolution;
        std::sort(neighborhood.begin(), neighborhood.end(), Move::bySavings); 

#ifdef COMPARE_OSRM
        compareCostWithOSRM(neighborhood);
#endif

        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {
	    current=currentSolution;
            current.v_applyMove(*it);
	    removedTruck = computeCosts(current);
	    newCost = current.getCost();
	    if (removedTruck) {
		currentSolution=current;
                bestSolution = current;
                computeCosts(bestSolution);
                bestSolutionCost = newCost;
                makeTabu(*it);
		STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
		//clear up all buckets
                neighborhood.clear(); 
                aspirationalTabu.clear(); notTabu.clear(); tabu.clear();
		return true;
            }

#ifdef LOG
	    std::cout<<"isTabu??: "<<(isTabu(*it)?"YES\n":"NO\n");it->Dump();
	    if (not ( (std::abs( (actualCost - newCost)  -  it->getsavings())) <0.5) ) {
		it->Dump();
		std::cout<<"something is wrong with the savings****** "<<it->getsavings()<< " ***** "<<actualCost - newCost<<"\n";
            } 
#endif
            // if the move is aspirational and not tabu then we apply it
            if (newCost  < bestSolutionCost and not isTabu(*it) ) {  
		applyAspirationalNotTabu(*it);
		//clear up all buckets
                neighborhood.clear(); 
                aspirationalTabu.clear(); notTabu.clear(); tabu.clear();
	        return true;
		
            }
	   // if the move is aspirational, but tabu, we save it
	   if (newCost  < bestSolutionCost and not foundAspTabu) { //save only the best aspirational Tabu
std::cout<<"newCost: "<<newCost;
std::cout<<"\tbestCost: "<<bestSolutionCost<<"\n";

		aspirationalTabu.push_back(*it);
		foundAspTabu = true;	
		//clear up remaining buckets 
                notTabu.clear(); tabu.clear();
	        switch (it->getmtype()) {
               		case Move::Ins: { 
				    neighborhood.clear(); 
				    return false; 
				}; 
               	//	case Move::IntraSw: { limit = number>limitIntraSw; }
               	//	case Move::InterSw: { limit = number>limitInterSw; }
        	};

		
	   } else if (not aspirationalTabu.size()  and not isTabu(*it) and not found ) { //save only the best nonTabu move when there is no aspirational tabu move
		notTabu.push_back(*it); 
		found = true;
		tabu.clear();  //no need to keep the tabus
		if (it->getsavings()<0) {neighborhood.clear(); return false; }  //we can stop now, no possible aspirational move can exist in this neighborhood
	    } else if (  not aspirationalTabu.size() and not notTabu.size() ) {  //all other buckets are empty so we save everything
		tabu.push_back(*it); 
            }	
        };
	neighborhood.clear();
	return false;         //we didnt make a move 
};
	
bool TabuOpt::computeCosts(OptSol &s) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::computeCosts \n";
#endif
        int removedTruck = s.v_computeCosts();
        if (removedTruck==-1) return false;
	removeTruckFromTabuList(removedTruck) ;
	return true;
}







    bool TabuOpt::dumpMoves(std::string str, std::deque<Move> moves) const {
	std::cout<<"Bucket: "<< str<<"\n";
	for (int i=0;i<moves.size();i++)
		moves[i].dump();
    };

