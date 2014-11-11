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

    //std::deque<Move> aspirationalTabu;
    //std::deque<Move> nonTabu;
    //std::deque<Move> tabu;
    currentIteration = 0;
    maxIteration = 1000;     // use ts.setMaxIteration(1) in trash.cpp

    Timer start;
    bool improvedBest;
    int lastImproved = 0;
    //first do a bunch of intersw moves
    doNeighborhoodMoves(InterSw, limitInterSw*5);

    do {
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration << std::endl;

        // this is a token ring search
        improvedBest = doNeighborhoodMoves(InterSw, 1 /*limitInterSw*5*/ );//, aspirationalTabu, nonTabu, tabu);
        improvedBest |= doNeighborhoodMoves(Ins, 1 /*limitIns*1*/);//, aspirationalTabu, nonTabu,tabu);
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
    bestSolution.tau();
}





void TabuOpt::getNeighborhood( neighborMovesName whichNeighborhood, Moves &neighborhood, double factor) const  {
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
/*if (neighborhood.size()) {
dumpMoves("interMoves",neighborhood);
std::cout<<"END of interMoves\n";
//assert(true==false);
}*/
                break;
        }
#ifdef TESTED
std::cout<<"Exiting TabuOpt::getNeighborhod() \n";
#endif
}

bool TabuOpt::applyAmove(const Move &move) {
#ifdef TESTED
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


/*
bool TabuOpt::applyAspirationalNotTabu(const Move  &move) {
	applyAmove(move); 			//has only one move
        STATS->inc("cnt Aspirational Not Tabu");
#ifndef TESTED
std::cout<<"\nAspirational non Tabu aplied ";move.Dump();
#endif
	return true;
}
*/

bool TabuOpt::applyMoves(std::string type, Moves &moves) {
	if (not moves.size()) return false;
#ifndef TESTED
std::cout<<"\napply moves: "<<type<<" applied: ";moves.begin()->Dump();
#endif
	applyAmove( *(moves.begin()) );
        STATS->inc("cnt "+type);
	cleanUpMoves( *(moves.begin()) );	

#ifndef LOG
std::cout<<"aspirational not Tabu size"<<aspirationalNotTabu.size()<<"\n";
std::cout<<"aspirationalTabu size"<<aspirationalTabu.size()<<"\n";
std::cout<<"not Tabu size"<<notTabu.size()<<"\n";
std::cout<<"Tabu size"<<tabu.size()<<"\n";
#endif
}
	

bool TabuOpt::reachedMaxCycles(int number, neighborMovesName whichNeighborhood) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::reachedMaxCycles "<<number<<"\n";
std::cout<<limitIns<<"\t"<<limitIntraSw<<"\t "<<limitInterSw<<"\n";
#endif
	bool limit;
        switch (whichNeighborhood) {
               case Ins: { limit = number>=limitIns; break;}
               case IntraSw: { limit = number>=limitIntraSw; break;}
               case InterSw: { limit = number>=limitInterSw; break;}
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

bool TabuOpt::doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxMoves) {

#ifdef TESTED
std::cout<<"Entering TabuOpt::doNeighobrhoodMoves\n";
#endif
    bool improvedBest = false;
    int Cnt = 0;
    int CntNonAspirational =0;
    int CntNoNeighborhood =0;
    double factor = 0.01;
    bool limit;
    int actualMoveCount=getTotalMovesMade();

    // we always start from the best solution of the last run
    //currentSolution = bestSolution;

    STATS->set("factor", factor);
    Moves neighborhood;

    do {
	std::cout<<"Factor:"<<factor<<"\n";
std::cout<<(getTotalMovesMade()-actualMoveCount)<<" > " <<maxMoves<<"***************************************************\n";
	if ((getTotalMovesMade()-actualMoveCount) > maxMoves) break;
        std::string solBefore = currentSolution.solutionAsText();

	if (factor==0.01 and CntNoNeighborhood==0 and notTabu.size()==0 and tabu.size()==0)
		getNeighborhood(whichNeighborhood,neighborhood,1); 
	else
		getNeighborhood(whichNeighborhood,neighborhood,factor); 
        Cnt++;
#ifdef LOG
std::cout<<"Just got a new neighborhood\n";
std::cout<<"neighborhood size"<<neighborhood.size()<<"\n";
std::cout<<"aspirationalNotTabu size"<<aspirationalNotTabu.size()<<"\n";
std::cout<<"aspirationalTabu size"<<aspirationalTabu.size()<<"\n";
std::cout<<"not Tabu size"<<notTabu.size()<<"\n";
std::cout<<"Tabu size"<<tabu.size()<<"\n";
#endif

	if (not neighborhood.size()) { 
	        //factor=std::min(factor+0.05,1.0); //need to increase the search space
	        factor=std::min(factor+1.0/limitInterSw,0.98); //need to increase the search space

		CntNoNeighborhood++; 
std::cout<<" No Moves are found  "<< CntNoNeighborhood <<"\n";
		if ( factor > 0.95 and reachedMaxCycles(CntNoNeighborhood,whichNeighborhood) ) {
std::cout<<" Reached end of cycle -No moves found- "<<Cnt<<" out of "<< maxMoves<<"\n";
		   return improvedBest; //we cycled and no neighborhood moves were found
                }; 
        };

	CntNoNeighborhood=0; 

        std::string solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);

	if ( classifyMoves(neighborhood)) {
		assert (not neighborhood.size());
		assert (not aspirationalTabu.size());
		assert (not notTabu.size());
		assert (not tabu.size());
		improvedBest=true;
		Cnt=0; //a move was made and it reduced the number of trucks
		continue;
        }/* else {
		if (aspirationalTabu.size() ) 	assert ( not notTabu.size() and not tabu.size() );
		if (notTabu.size() ) assert ( not aspirationalTabu.size() and not tabu.size() );
		if (tabu.size() ) assert ( not aspirationalTabu.size() and not notTabu.size() );
	}*/

#ifndef LOG
std::cout<<"after Classify Moves\n";
std::cout<<"neighborhood size"<<neighborhood.size()<<"\n";
std::cout<<"aspirational not Tabu size"<<aspirationalNotTabu.size()<<"\n";
std::cout<<"aspirationalTabu size"<<aspirationalTabu.size()<<"\n";
std::cout<<"not Tabu size"<<notTabu.size()<<"\n";
std::cout<<"Tabu size"<<tabu.size()<<"\n";
#endif
	if ( aspirationalNotTabu.size() ) {
		improvedBest=true;
		// factor=std::max(factor-0.05,0.1); //found non tabu & aspirational  I am optimistic
		//factor=std::max(factor-0.01,0.01); //found non tabu & aspirational  I am optimistic
		factor=0.01; //trully tryllu optimistic
		while (aspirationalNotTabu.size()) {
			aspirationalTabu.clear();notTabu.clear(); tabu.clear();
			applyMoves("aspirational non tabu",aspirationalNotTabu);
			Cnt=0;
		}
		continue;
	}	

	

	if ( aspirationalTabu.size() ) {
		improvedBest=true;
		//factor=std::max(factor-0.01,0.01); //found non tabu & aspirational  I am optimistic
		factor=0.01; //trully tryllu optimistic
		while ( aspirationalTabu.size() ) {
			notTabu.clear(); tabu.clear();
                	applyMoves("aspirational Tabu",  aspirationalTabu );
			Cnt=0;
		}
		continue;
        }
	//factor=std::min(factor+0.05,1.0); //otherwise I am pesimistic & need to increase the search space
	factor=std::min(factor+1.0/limitInterSw,0.98); //need to increase the search space

        if (not reachedMaxCycles(Cnt,whichNeighborhood) and not notTabu.size() ) continue;


	if (notTabu.size() and ( (notTabu.begin()->getsavings()>=0) or reachedMaxCycles(Cnt,whichNeighborhood) ) ) {
		while ( notTabu.size() ) {
			tabu.clear();
dumpMoves("notTabu",notTabu);
			applyMoves("not Tabu",  notTabu );
			if (notTabu.begin()->getsavings()<0) notTabu.clear(); //after aplying 1, only apply positives 
			Cnt=0;
		}
		continue;
	} 

        if (not reachedMaxCycles(Cnt,whichNeighborhood)) continue;

        solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);

	while ( tabu.size() ) {
		applyMoves("tabu", tabu ); //best
		Cnt=0;
        }
         
    }	
    while (  (getTotalMovesMade()-actualMoveCount) < maxMoves );

#ifndef TESTED
std::cout<<" Moves made "<<Cnt<<" out of "<< maxMoves<<"\n";
std::cout<<"Exiting TabuOpt::doNeighobrhoodMoves\n";
#endif
    return improvedBest;
}

#ifndef TESTED
void TabuOpt::compareCostWithOSRM(Moves &neighborhood) {
    int cnt = 0;
    Vehicle truckFrom, truckFromAfter;
    Vehicle truckTo, truckToAfter;
    Vehicle truck1, truck1After;
    Vehicle truck2, truck2After;
    double c_truckFrom,c_truckTo,co_truckFrom,co_truckTo,c_truckFromAfter,c_truckToAfter,co_truckFromAfter,co_truckToAfter,t_save,o_save,c_truck1,co_truck1,c_truck1After,co_truck1After,c_truck2,co_truck2,c_truck2After,co_truck2After;

    for (MovesItr it=neighborhood.begin();
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
                truckFrom.evaluateOsrm();

                truckTo   = currentSolution[it->getInsToTruck()];
                truckTo.evaluateOsrm();

                truckFromAfter = current[it->getInsFromTruck()];
                truckFromAfter.evaluateOsrm();

                truckToAfter   = current[it->getInsToTruck()];
                truckToAfter.evaluateOsrm();

                c_truckFrom       = truckFrom.getcost();
                c_truckTo         = truckTo.getcost();
                co_truckFrom      = truckFrom.getCostOsrm();
                co_truckTo        = truckTo.getCostOsrm();
                c_truckFromAfter  = truckFromAfter.getcost();
                c_truckToAfter    = truckToAfter.getcost();
                co_truckFromAfter = truckFromAfter.getCostOsrm();
                co_truckToAfter   = truckToAfter.getCostOsrm();
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
                truck1.evaluateOsrm();

                truck1After = current[it->getIntraSwTruck()];
                truck1After.evaluateOsrm();

                c_truck1       = truck1.getcost();
                co_truck1      = truck1.getCostOsrm();
                c_truck1After  = truck1After.getcost();
                co_truck1After = truck1After.getCostOsrm();
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
                truck1.evaluateOsrm();

                truck2 = currentSolution[it->getInterSwTruck2()];
                truck2.evaluateOsrm();

                truck1After = current[it->getInterSwTruck1()];
                truck1After.evaluateOsrm();

                truck2After = current[it->getInterSwTruck2()];
                truck2After.evaluateOsrm();

                c_truck1       = truck1.getcost();
                c_truck2       = truck2.getcost();
                co_truck1      = truck1.getCostOsrm();
                co_truck2      = truck2.getCostOsrm();
                c_truck1After  = truck1After.getcost();
                c_truck2After  = truck2After.getcost();
                co_truck1After = truck1After.getCostOsrm();
                co_truck2After = truck2After.getCostOsrm();
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
#endif

/**
  Classify the moves into:
	aspirational Not tabu	(if found the buckets bellow are cleared)
	aspirational tabu	(adds to the bucket the best move)
	not Tabu		(adds to the bucket the best move) 
	Tabu			(adds to the bucket all the moves)

  if during the classification a truck is removed:
	move is applied
	all buckets are cleared


	returns true: the truck number was reduced
*/
bool TabuOpt::classifyMoves(Moves &neighborhood) {
#ifndef TESTED
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
	Move guide;
        //std::sort(neighborhood.begin(), neighborhood.end(), Move::bySavings); 

#ifdef COMPARE_OSRM
        compareCostWithOSRM(neighborhood);
#endif

        for (MovesItr it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {
	    current=currentSolution;
            current.v_applyMove(*it);
	    removedTruck = computeCosts(current);
	    newCost = current.getCost();
	    if (removedTruck) {
		currentSolution=current;
                setCurrentAsBest();
                makeTabu(*it);
		STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
		//clear up all buckets
                neighborhood.clear(); 
                aspirationalNotTabu.clear();aspirationalTabu.clear(); notTabu.clear(); tabu.clear();
		return true;
            }

#ifdef LOG
	    std::cout<<"isTabu??: "<<(isTabu(*it)?"YES\n":"NO\n");it->Dump();
	    if (not ( (std::abs( (actualCost - newCost)  -  it->getsavings())) <0.5) ) {
		it->Dump();
		std::cout<<"something is wrong with the savings****** "<<it->getsavings()<< " ***** "<<actualCost - newCost<<"\n";
            } 
#endif
            if (newCost  < bestSolutionCost and not isTabu(*it) ) aspirationalNotTabu.insert(*it);
            else if (newCost  < bestSolutionCost )  aspirationalTabu.insert(*it);
	    else if ( not isTabu(*it) ) notTabu.insert(*it); 
	    else tabu.insert(*it); 
            	
        };
#ifdef LOG
	std::cout<<" Neighborhood size:  "<< neighborhood.size() <<"\n";
	std::cout<<" aspirational not Tabu size:  "<< aspirationalNotTabu.size() <<"\n";
	std::cout<<" aspirational Tabu size:  "<< aspirationalTabu.size() <<"\n";
	std::cout<<" not Tabu size:  "<< notTabu.size() <<"\n";
	std::cout<<" Tabu size:  "<< tabu.size() <<"\n";
#endif
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


void TabuOpt::cleanUpInterSwMoves(Moves &moves, const Move &guide ) const  {
	if (not moves.size() ) return;
	if (not guide.getmtype()==Move::InterSw) return;
	if (not moves.begin()->getmtype()==Move::InterSw) return;
	int fromPos = guide.getInterSwFromPos();
	int toPos = guide.getInterSwToPos();
	Moves oldMoves=moves;
	moves.clear();
	Move move;
	for(MovesItr movePtr=oldMoves.begin(); movePtr!=oldMoves.end();++movePtr) {
		move = (*movePtr);
		if ( guide.isTabu(move) ) continue;
		else if ( guide.getnid1() == move.getnid1() or  guide.getnid1() == move.getnid2()
		     or guide.getnid2() == move.getnid2() or  guide.getnid2() == move.getnid1() )
			continue;
		else if ( (move.getInterSwFromPos() < fromPos-1 or move.getInterSwFromPos() > fromPos+1) 
		   and (move.getInterSwToPos() < toPos-1 or move.getInterSwToPos() > toPos+1) ) {
			//reinsert only if feasable
			if ( currentSolution.testInterSwMove(move) ) moves.insert(move);
                }
        }
        std::cout<<"oldMoves size"<<oldMoves.size()<<"\n";
        std::cout<<"newMoves size"<<moves.size()<<"\n";
        //dumpMoves("newMoves",moves);
}	


void TabuOpt::cleanUpInsMoves(Moves &moves, const Move &guide )  {
        if (not moves.size() ) return;
        if (not guide.getmtype()==Move::Ins) return;
        if (not moves.begin()->getmtype()==Move::Ins) return;
        int fromPos = guide.getInsFromPos();
        int toPos = guide.getInsToPos();
        Moves oldMoves=moves;
        moves.clear();
        Move move;
        for(MovesItr movePtr=oldMoves.begin(); movePtr!=oldMoves.end();++movePtr) {
                move = (*movePtr);
//std::cout<<"\t";move.Dump();
                if (move.getsavings()<0) break;
		if (move==guide) continue;
		if (move.getInsFromPos() == fromPos) continue; // that container is not longer there
		if (move.getInsFromPos() > fromPos) move.setInsFromPos(move.getInsFromPos()-1); //the container was shifted -1 position 
		if (move.getInsToPos() == toPos) continue; //the evaluation is not longer valid
		if (move.getInsToPos() > toPos) move.setInsToPos(move.getInsToPos()+1); //the container was shifted 1 position 
		 
                //insert only if feasable
                if ( currentSolution.testInsMove(move) ) moves.insert(move);
                
        }
        std::cout<<"oldMoves size"<<oldMoves.size()<<"\n";
        std::cout<<"newMoves size"<<moves.size()<<"\n";
        //dumpMoves("newMoves",moves);
}

void TabuOpt::cleanUpMoves(const Move &guide ) {
	switch (guide.getmtype()){
		case Move::InterSw: 
			cleanUpInterSwMoves(aspirationalNotTabu,guide);
			cleanUpInterSwMoves(aspirationalTabu,guide);
			cleanUpInterSwMoves(notTabu,guide);
			cleanUpInterSwMoves(tabu,guide);
			break;
		case Move::Ins: 
        if (aspirationalNotTabu.size()) std::cout<<"cleaning aspirational not tabu\n";
			cleanUpInsMoves(aspirationalNotTabu,guide);
        if (aspirationalTabu.size()) std::cout<<"cleaning aspirational tabu\n";
			cleanUpInsMoves(aspirationalTabu,guide);
        if (notTabu.size()) std::cout<<"cleaning not tabu\n";
			cleanUpInsMoves(notTabu,guide);
        if (tabu.size()) std::cout<<"cleaning tabu\n";
			cleanUpInsMoves(tabu,guide);
			break;
	}
}




    bool TabuOpt::dumpMoves(std::string str, Moves moves) const {
	std::cout<<"Bucket: "<< str<<"\n";
	MovesItr movePtr;
	for(movePtr=moves.begin(); movePtr!=moves.end();++movePtr)
		movePtr->Dump();
    };

