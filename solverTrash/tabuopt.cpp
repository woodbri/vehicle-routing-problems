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

#include "optsol.h"
#include "tabuopt.h"
#ifdef DOSTATS
#include "stats.h"
#endif

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
#ifdef DOSTATS
 STATS->inc("TabuOpt::search ");
#endif
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
    //first do a bunch of intersw/Intrasw  moves
/*
    for (int i=0;i<limitInterSw*3;i++) {
        doNeighborhoodMoves(Move::InterSw, 1, Move::InterSw);
        doNeighborhoodMoves(Move::IntraSw, 1, Move::InterSw);
    }
*/
    for (int i=0; i< maxIteration; i++) {
	#ifndef LOG
	start.restart();
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration << std::endl;
	#endif
        // this is a token ring search
	setBestAsCurrent();
	
        improvedBest = doNeighborhoodMoves(Move::InterSw, 1 , Move::InterSw );
//        improvedBest |= doNeighborhoodMoves(Move::IntraSw, 1, Move::InterSw);
//        improvedBest |= doNeighborhoodMoves(Move::Ins, 1 ,    Move::Ins);
//        improvedBest |= doNeighborhoodMoves(Move::IntraSw, 1, Move::Ins);

        if (improvedBest) lastImproved = 0;
        else ++lastImproved;

	#ifndef LOG
        std::cout << "TABUSEARCH: Finished iteration: " << currentIteration
            << ", improvedBest: " << improvedBest
            << ", run time in seconds: " << start.duration()
            << std::endl;
	#endif

	#ifdef DOSTATS
        STATS->set("Iteration", currentIteration);
        STATS->set("Best Cost After", bestSolution.getCost());
        dumpStats();
        std::cout << "--------------------------------------------\n";
	#endif
	currentIteration++;
	if (not improvedBest) break; //propably we need to shake a little and restart all over
    }

    std::cout << "TABUSEARCH: Total time: " << start.duration() << std::endl;
    bestSolution.tau();
}





void TabuOpt::getNeighborhood(  Move::Mtype  whichNeighborhood, Moves &neighborhood, double factor, Move::Mtype mtype) const  {
        neighborhood.clear();
#ifdef DOSTATS
 STATS->inc("TabuOpt::getNeighborhood ");
#endif
#ifdef TESTED
std::cout<<"Entering TabuOpt::getNeighborhod() \n";
#endif
        Timer getNeighborhoodTimer;
        switch (whichNeighborhood) {
            case Move::Ins:
                currentSolution.getInsNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("Ins", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
            case Move::IntraSw:
                currentSolution.getIntraSwNeighborhood(mtype, neighborhood, factor);
                generateNeighborhoodStats("IntraSw", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
            case Move::InterSw:
                currentSolution.getInterSwNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("InterSw", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
        }
}


bool TabuOpt::applyAmove(const Move &move) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::applyAmove ");
#endif
#ifdef TESTED
std::cout<<"\nApply a move ";move.Dump();
#endif

        currentSolution.v_applyMove(move); 
        makeTabu(move);
        computeCosts(currentSolution);
        currentCost=currentSolution.getCost();
        if (bestSolutionCost > currentCost) {
		setCurrentAsBest();
		#ifdef DOSTATS
        	STATS->set("best Updated Last At Iteration", currentIteration);
        	STATS->inc("number of times best was Updated ");
		#endif
	}
        return true;
}


bool TabuOpt::applyMoves(std::string type, Moves &moves) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::applyMoves ");
#endif
	if (not moves.size()) return false;
#ifndef TESTED
std::cout<<"\napply moves: "<<type<<" applied: ";moves.begin()->Dump();
#endif
	applyAmove( *(moves.begin()) );
	#ifdef DOSTATS
        STATS->inc("Number of moves of type "+type);
	#endif
	cleanUpMoves( *(moves.begin()) );	

}
	

bool TabuOpt::reachedMaxCycles(int number,  Move::Mtype whichNeighborhood) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::reachedMaxCycles ");
#endif
#ifndef TESTED
std::cout<<"Entering TabuOpt::reachedMaxCycles "<<number<<"\n";
#endif
	bool limit;
        switch (whichNeighborhood) {
               case Move::Ins: { limit = number>=limitIns; break;}
               case Move::IntraSw: { limit = number>=limitIntraSw; break;}
               case Move::InterSw: { limit = number>=limitInterSw; break;}
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

bool TabuOpt::doNeighborhoodMoves( Move::Mtype whichNeighborhood, int maxMoves, Move::Mtype mtype) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::doNeighborhoodMoves ");
#endif

#ifdef TESTED
std::cout<<"Entering TabuOpt::doNeighobrhoodMoves\n";
#endif
    bool improvedBest = false;
    int Cnt = 0;
    int CntNonAspirational =0;
    int CntNoNeighborhood =0;
    double factor = 0.02;
    bool limit;
    int actualMoveCount=getTotalMovesMade();
    bool moveMade=false;
    bool intraSwMoveMade=false;


    STATS->set("factor", factor);
    Moves neighborhood;

currentSolution.tau();
    do {
#ifndef LOG
std::cout<<(getTotalMovesMade()-actualMoveCount)<<" > " <<maxMoves<<"***************************************************\n";
std::cout<<" Factor  "<< factor <<"\n";
#endif
	if ((getTotalMovesMade()-actualMoveCount) > maxMoves) break;


	if (factor==0.01 and CntNoNeighborhood==0 and notTabu.size()==0 and tabu.size()==0)
		getNeighborhood(whichNeighborhood,neighborhood,1,mtype); 
	else
		getNeighborhood(whichNeighborhood,neighborhood,factor,mtype); 
        Cnt++;

	if (not neighborhood.size()) { 
	        factor=std::min(factor+1.0/limitInterSw,factor+0.1); //need to increase the search space

		CntNoNeighborhood++; 
#ifndef LOG
std::cout<<" No Moves are found  "<< CntNoNeighborhood <<"\n";
std::cout<<" Factor  "<< factor <<"\n";
#endif
		if ( reachedMaxCycles(CntNoNeighborhood,whichNeighborhood) or whichNeighborhood==Move::IntraSw) {
#ifndef LOG
std::cout<<" Reached end of cycle - for No moves found- "<<Cnt<<" out of "<< maxMoves<<"\n";
#endif
		   return improvedBest; //we cycled and no neighborhood moves were found
                }; 
		if (not notTabu.size() and not tabu.size() ) continue;
        } else 	CntNoNeighborhood=0; 

        std::string solAfter  = currentSolution.solutionAsText();

	if ( classifyMoves(neighborhood)) {
		assert (not neighborhood.size());
		assert (not aspirationalTabu.size());
		assert (not notTabu.size());
		assert (not tabu.size());
		improvedBest=true;
		moveMade=true;
		Cnt=0; //a move was made and it reduced the number of trucks
		continue;
        }

	if ( aspirationalNotTabu.size() ) {
		improvedBest=true;
		moveMade=true;
		factor=0.01; //trully tryllu optimistic
		while (aspirationalNotTabu.size()) { //do as many as the bookkeeping allows
			applyMoves("aspirational non tabu",aspirationalNotTabu);
			Cnt=0;
		}
	}	

	

	if ( aspirationalTabu.size() ) {
		improvedBest=true;
		moveMade=true;
		factor=0.01; //trully tryllu optimistic
		while ( aspirationalTabu.size() ) { //do as many as the bookkeping allows
                	applyMoves("aspirational Tabu",  aspirationalTabu );
			Cnt=0;
		}
        }

	factor=std::min(factor+1.0/limitInterSw,factor+0.1); //need to increase the search space

	if (notTabu.size() and  notTabu.begin()->getsavings()>=0)   {
		improvedBest=true;
		moveMade=true;
		while ( notTabu.size() ) {
			applyMoves("not Tabu with pos savings",  notTabu );
			if (notTabu.begin()->getsavings()<0) notTabu.clear(); // only apply positives 
			Cnt=0;
		}
	} 

	if (moveMade==true) break;
std::cout<<"1\n";
	if (not (whichNeighborhood==Move::IntraSw)) {
std::cout<<"2\n";
        improvedBest =  doNeighborhoodMoves(Move::IntraSw, 1, Move::InterSw);
	intraSwMoveMade=true;
	if (improvedBest)  actualMoveCount=getTotalMovesMade();
	std::cout<<"aspirational not Tabu size"<<aspirationalNotTabu.size()<<"\n";dumpMoves("aspirationalNotTabu",aspirationalNotTabu);
	std::cout<<"aspirationalTabu size"<<aspirationalTabu.size()<<"\n";dumpMoves("aspirationalTabu",aspirationalTabu);
	std::cout<<"not Tabu size"<<notTabu.size()<<"\n";dumpMoves("notTabu",notTabu);
	std::cout<<"Tabu size"<<tabu.size()<<"\n";dumpMoves("tabu",tabu);
	//assert(true==false);
	}
std::cout<<"3\n";

	if (notTabu.size() and factor>0.9 and reachedMaxCycles(Cnt,whichNeighborhood) )  {
		while ( notTabu.size() ) {
			applyMoves("not Tabu",  notTabu );
			if (notTabu.begin()->getsavings()<0) notTabu.clear(); //after aplying 1, only apply positives 
			Cnt=0;
		}
		moveMade=true;
		continue;
	} 

        if (factor<= 0.9  and not reachedMaxCycles(Cnt,whichNeighborhood)) continue;

	if (intraSwMoveMade) break;
	while ( tabu.size() ) {
		applyMoves("tabu", tabu ); //best
		if (tabu.begin()->getsavings()<0) tabu.clear(); //after aplying 1, only apply positives 
		Cnt=0;
		moveMade=true;
        }
         
    }	
    while ( not moveMade );
//or (getTotalMovesMade()-actualMoveCount) < maxMoves );

#ifndef LOG
std::cout<<" Moves made "<<Cnt<<" out of "<< maxMoves<<"\n";
#endif
    return improvedBest;
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


	returns true: the truck number was reduced
*/
bool TabuOpt::classifyMoves(Moves &neighborhood) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::classifyMoves ");
#endif
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
	Move guide;

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
		#ifdef DOSTATS
		STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
		#endif
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
	neighborhood.clear();
	return false;         //we didnt make a move that deleted a truck
};
	
bool TabuOpt::computeCosts(OptSol &s) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::computeCosts ");
#endif
        int removedTruck = s.v_computeCosts();
        if (removedTruck==-1) return false;
	removeTruckFromTabuList(removedTruck) ;
	return true;
}


void TabuOpt::cleanUpInterSwMoves(Moves &moves, const Move &guide ) const  {
#ifdef DOSTATS
 STATS->inc("TabuOpt::cleanUpInterSwMoves ");
#endif
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
}	


void TabuOpt::cleanUpInsMoves(Moves &moves, const Move &guide )  {
#ifdef DOSTATS
 STATS->inc("TabuOpt::cleanUpInsMoves ");
#endif
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
                if (move.getsavings()<0) break;
		if (move==guide) continue;
		if (move.getInsFromPos() == fromPos) continue; // that container is not longer there
		if (move.getInsFromPos() > fromPos) move.setInsFromPos(move.getInsFromPos()-1); //the container was shifted -1 position 
		if (move.getInsToPos() == toPos) continue; //the evaluation is not longer valid
		if (move.getInsToPos() > toPos) move.setInsToPos(move.getInsToPos()+1); //the container was shifted 1 position 
		 
                if ( currentSolution.testInsMove(move) ) moves.insert(move);
                
        }
}



void TabuOpt::cleanUpIntraSwMoves(Moves &moves, const Move &guide ) const  {
#ifdef DOSTATS
 STATS->inc("TabuOpt::cleanUpIntraSwMoves ");
#endif

        if (not moves.size() ) return;
        Moves oldMoves=moves;
        moves.clear();
        Move move;
        if (not (guide.getmtype()==Move::IntraSw)) return;
        int truckPos = guide.getIntraSwTruck();
        for(MovesItr movePtr=oldMoves.begin(); movePtr!=oldMoves.end();++movePtr) {
                move = (*movePtr);
                if (not (move.getmtype()==Move::IntraSw)) continue;
                if ( truckPos == move.getIntraSwTruck() ) continue;
		if ( move.getIntraSwNid1() == guide.getIntraSwNid1() ) continue;
		if ( move.getIntraSwNid2() == guide.getIntraSwNid2() ) continue;
                moves.insert(move);
        }
}




void TabuOpt::cleanUpMoves(const Move guide ) {
#ifdef DOSTATS
 STATS->inc("TabuOpt::cleanUpMoves ");
#endif
	#ifdef LOG
	std::cout<<"ENTERING TabuOpt::cleanUpMoves\n";
	if (guide.getmtype()==Move::IntraSw) {
	guide.Dump();
        if (aspirationalNotTabu.size()) std::cout<<"cleaning aspirational not tabu\n";
        if (aspirationalTabu.size()) std::cout<<"cleaning aspirational tabu\n";
        if (notTabu.size()) std::cout<<"cleaning not tabu\n";
        if (tabu.size()) std::cout<<"cleaning tabu\n";
	std::cout<<"aspirational not Tabu size"<<aspirationalNotTabu.size()<<"\n";dumpMoves("aspirationalNotTabu",aspirationalNotTabu);
	std::cout<<"aspirationalTabu size"<<aspirationalTabu.size()<<"\n";dumpMoves("aspirationalTabu",aspirationalTabu);
	std::cout<<"not Tabu size"<<notTabu.size()<<"\n";dumpMoves("notTabu",notTabu);
	std::cout<<"Tabu size"<<tabu.size()<<"\n";dumpMoves("tabu",tabu);
	}
	#endif
	switch (guide.getmtype()){
		case Move::InterSw: 
			if( aspirationalNotTabu.size()) cleanUpInterSwMoves(aspirationalNotTabu,guide);
			if( aspirationalTabu.size()) cleanUpInterSwMoves(aspirationalTabu,guide);
			if( notTabu.size()) cleanUpInterSwMoves(notTabu,guide);
			if (tabu.size()) cleanUpInterSwMoves(tabu,guide);
			break;
		case Move::IntraSw: 
			if (aspirationalNotTabu.size()) cleanUpIntraSwMoves(aspirationalNotTabu,guide);
			if( aspirationalTabu.size()) cleanUpIntraSwMoves(aspirationalTabu,guide);
			if (notTabu.size()) cleanUpIntraSwMoves(notTabu,guide);
			if (tabu.size()) cleanUpIntraSwMoves(tabu,guide);
			break;
		case Move::Ins: 
			if( aspirationalNotTabu.size()) cleanUpInsMoves(aspirationalNotTabu,guide);
			if (aspirationalTabu.size()) cleanUpInsMoves(aspirationalTabu,guide);
			if (notTabu.size())cleanUpInsMoves(notTabu,guide);
			if (tabu.size()) cleanUpInsMoves(tabu,guide);
			break;
	}
	#ifdef LOG
	std::cout<<"aspirational not Tabu size"<<aspirationalNotTabu.size()<<"\n";dumpMoves("aspirationalNotTabu",aspirationalNotTabu);
	std::cout<<"aspirationalTabu size"<<aspirationalTabu.size()<<"\n";dumpMoves("aspirationalTabu",aspirationalTabu);
	std::cout<<"not Tabu size"<<notTabu.size()<<"\n";dumpMoves("notTabu",notTabu);
	std::cout<<"Tabu size"<<tabu.size()<<"\n";dumpMoves("tabu",tabu);
	#endif
}




    bool TabuOpt::dumpMoves(std::string str, Moves moves) const {
#ifdef DOSTATS
 STATS->inc("TabuOpt::dumpMoves ");
#endif
	std::cout<<"Bucket: "<< str<<"\n";
	MovesItr movePtr;
	for(movePtr=moves.begin(); movePtr!=moves.end();++movePtr)
		movePtr->Dump();
    };

