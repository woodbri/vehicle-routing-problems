
#include <iostream>
#include <deque>
#include <algorithm>

#include "trashstats.h"
#include "neighborhoods.h"
#include "tabusearch.h"

void TabuSearch::dumpTabuList() const {
    std::map<const Move, int>::const_iterator it;

    std::cout << "TabuList at iteration: " << currentIteration << std::endl;
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
        it->first.dump();
        std::cout << " - expires: " << it->second << std::endl;
    }
    std::cout << "--------------------------" << std::endl;
}


void TabuSearch::dumpStats() const {
    std::cout << "TabuList Stats at iteration: " << currentIteration << std::endl;
    STATS->dump("");
}


bool TabuSearch::isTabu(const Move& m) const {
    std::map<const Move, int>::const_iterator it;

    STATS->inc("tabu Moves Checked");
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
        if (it->second < currentIteration) continue;
        if (m.isForbidden(it->first)) {
            STATS->inc("tabu Moves Checked Tabu");
            return true;
        }
    }
    return false;
}


void TabuSearch::cleanExpired() {
    std::map<const Move, int>::iterator it;
    for (it = TabuList.begin(); it!=TabuList.end(); ++it)
        if (it->second < currentIteration)
            TabuList.erase(it);
}


void TabuSearch::makeTabu(const Move m) {
    int r = rand()%5-2;
    switch (m.getmtype()) {
        case Move::Ins:
            TabuList[m] = currentIteration + tabuLengthIns + r;
            STATS->inc("tabu Ins Moves Added");
            break;
        case Move::IntraSw:
            TabuList[m] = currentIteration + tabuLengthIntraSw + r;
            STATS->inc("tabu IntraSw Moves Added");
            break;
        case Move::InterSw:
            TabuList[m] = currentIteration + tabuLengthInterSw + r;
            STATS->inc("tabu InterSw Moves Added");
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

void TabuSearch::search() {

    currentIteration = 0;

    Timer start;
    bool madeChanges;
    do {
        // we always start from the best solution of the last run
        currentSolution = bestSolution;

        // set the stagnation count as the last the last parameter
        // thhe values 500, 300, 300 can from the paper mentioned above
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration
            << std::endl;

        // this is a token ring search
        // for each iteration we process all moves available
        // from each slot (ie: Ins, IntraSw, InterSw)
        // when there are no move moves we terminate
        madeChanges = doNeighborhoodMoves(Ins,     500)
                    | doNeighborhoodMoves(IntraSw, 300)
                    | doNeighborhoodMoves(InterSw, 300)
                    ;

        std::cout << "TABUSEARCH: Finished iteration: " << currentIteration
            << ", madeChanges: " << madeChanges << std::endl;

        STATS->set("0 Iteration", currentIteration);
        STATS->set("0 Best Cost After", bestSolution.getCost());
        dumpStats();
        std::cout << "--------------------------------------------\n";
    }
    while (madeChanges and ++currentIteration < maxIteration);

    std::cout << "TABUSEARCH: Total time: " << start.duration() << std::endl;
}



/*
    Algorithm for processing neighborhood moves

    For the requested individual move neighborhood:
        doInsMoves, doIntraSwMoves, doInterSwMoves

    do {
        Generate the neighhood of moves order from best to worst.
        Working through the neighborhood (best to worst)
        Filter out inFeasible moves then
        if the move is aspirational we apply the move
        otherwise if the move is not Tabu apply the move
        even if it makes the solution worse.
    } until there are not valid moves or stagnation 
    return an indicator that we made changes or not.
*/

bool TabuSearch::doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxStagnation) {
    bool madeMove;
    bool loopMadeMove;
    int stagnationCnt = 0;
    double factor = 0.5;
    madeMove = false;

    STATS->set("factor", factor);

    do {
        loopMadeMove = false;

        // generate the a move neighborhood based on the currentSolution
        // this is fast and stupid move generation because we will
        // only look at a very small percentage of the actual moves
        // because we have to calcuate savings we should have already
        // checked for feasibility so only feasible solutions are returned

        // VERIFY that  neighborhood generation is CONST
        std::string solBefore = currentSolution.solutionAsText();

        std::deque<Move> neighborhood;
        Timer timeNeighboorhoodGeneration;
        switch (whichNeighborhood) {
            case Ins:
                currentSolution.v_getInsNeighborhood(neighborhood, factor);
                //currentSolution.getInsNeighborhood(neighborhood);
                STATS->addto("time Gen Ins", timeNeighboorhoodGeneration.duration());
                STATS->inc("cnt Gen Ins Calls");
                STATS->addto("cum Ins Moves", neighborhood.size());
std::cout << "\tdoNeighborhoodMoves for Ins: " << neighborhood.size()
    << " moves generated" << std::endl;
                break;
            case IntraSw:
                //currentSolution.v_getIntraSwNeighborhood(neighborhood, factor);
                currentSolution.getIntraSwNeighborhood(neighborhood);
                STATS->addto("time Gen IntraSw", timeNeighboorhoodGeneration.duration());
                STATS->inc("cnt Gen IntraSw Calls");
                STATS->addto("cum IntraSw Moves", neighborhood.size());
std::cout << "\tdoNeighborhoodMoves for IntraSw: " << neighborhood.size()
    << " moves generated" << std::endl;
                break;
            case InterSw:
                currentSolution.v_getInterSwNeighborhood(neighborhood, factor);
                //currentSolution.getInterSwNeighborhood(neighborhood);
                STATS->addto("time Gen InterSw", timeNeighboorhoodGeneration.duration());
                STATS->inc("cnt Gen InterSw Calls");
                STATS->addto("cum InterSw Moves", neighborhood.size());
std::cout << "\tdoNeighborhoodMoves for InterSw: " << neighborhood.size()
    << " moves generated" << std::endl;
                break;
        }

        // VERIFY that neighborhood generation is CONST
        std::string solAfter = currentSolution.solutionAsText();
        assert(solBefore == solAfter);

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
                bestSolution = currentSolution;
                bestSolutionCost = bestSolution.getCost();
                makeTabu(*it);
                loopMadeMove = true;
                stagnationCnt = 0;
                allTabu = false;
                STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
                // update stats
                switch (whichNeighborhood) {
                    case Ins:     STATS->inc("cnt Ins Applied");    break;
                    case IntraSw: STATS->inc("cnt IntraSw Applied"); break;
                    case InterSw: STATS->inc("cnt InterSw Applied"); break;
                }

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
                loopMadeMove = true;
                allTabu = false;
                // update stats
                switch (whichNeighborhood) {
                    case Ins:     STATS->inc("cnt Ins Applied");    break;
                    case IntraSw: STATS->inc("cnt IntraSw Applied"); break;
                    case InterSw: STATS->inc("cnt InterSw Applied"); break;
                }

                // ok we made a move, so now the neighborhood is no
                // longer valid so break to regenerate a new neighborhood
                break;
            }
            else {
                ++stagnationCnt;
            }
        }
        if (allTabu and neighborhood.size()) {
std::cout << "\tdoNeighborhoodMoves: All Tabu: "; neighborhood.begin()->dump();
            currentSolution.applyMove(*(neighborhood.begin()));
            makeTabu(*(neighborhood.begin()));
            madeMove = true;
            switch (whichNeighborhood) {
                case Ins:     STATS->inc("cnt allTabu Ins Applied");    break;
                case IntraSw: STATS->inc("cnt allTabu IntraSw Applied"); break;
                case InterSw: STATS->inc("cnt allTabu InterSw Applied"); break;
            }
        }
        STATS->addto("time Apply Moves", applyMoveTimer.duration());
        madeMove = madeMove or loopMadeMove;
    }
    while (madeMove and stagnationCnt < maxStagnation );

    if (not madeMove)
        std::cout << "\tStagnation reached in neighborhood: " << whichNeighborhood << std::endl;

    return madeMove;
}

///////////////////////////////////////////////////

void TabuSearch::v_search() {
#ifndef TESTED
std::cout<<"Entering TabuSearch::v_search() \n";
#endif

    currentIteration = 0;

    bool madeChanges;
    do {
        // set the stagnation count as the last the last parameter
        // thhe values 500, 300, 300 can from the paper mentioned above
        madeChanges = //v_doNeighborhoodMoves(Ins,     500);
                  v_doNeighborhoodMoves(IntraSw, 300);
                //  v_doNeighborhoodMoves(InterSw, 300);
		assert(true==false);
    }
    while (madeChanges and ++currentIteration < maxIteration);

}



bool TabuSearch::v_doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxStagnation) {
#ifndef TESTED
std::cout<<"Entering TabuSearch::v_doNeighborhoodMoves() \n";
#endif
    bool madeMove;
    bool loopMadeMove;
    int stagnationCnt = 0;
    double factor=0.5;
    madeMove = false;

    do {
        loopMadeMove = false;

        // generate the a move neighborhood based on the currentSolution
        // this is fast and stupid move generation because we will
        // only look at a very small percentage of the actual moves
        // because we have to calcuate savings we should have already
        // checked for feasibility so only feasible solutions are returned
        std::deque<Move> neighborhood;
        std::deque<Move> setOfInsMoves;
        switch (whichNeighborhood) {
/*            case Ins:

std::cout<<"TabuSearch::v_doNeighborhoodMoves 1\n";
                currentSolution.v_getInsNeighborhood(setOfInsMoves,factor);
std::cout<<"TabuSearch::v_doNeighborhoodMoves >> \n"<<setOfInsMoves.size()<<"moves found";
		assert(true==false);
                break;
*/            case IntraSw:
                currentSolution.v_getIntraSwNeighborhood(neighborhood,factor);
                break;
/*            case InterSw:
                currentSolution.v_getInterSwNeighborhood(neighborhood,factor);
                break;
*/        }

        // and sort it so we can work from the best to the worst
        std::sort(neighborhood.begin(), neighborhood.end(), Move::bySavings);
 currentSolution.plot("before","bintraSwp");
neighborhood[0].dump();
 currentSolution.applyIntraSwMove(neighborhood[0]);
 currentSolution.plot("zafter","bintraSwp");


std::cout<<"======================================================";
	for (int i=0;i<neighborhood.size();i++) neighborhood[i].dump();
std::cout<<"======================================================";
		assert(true==false);

        // take the best move that we may apply and apply it, if any
/*
        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {

            // if the move is aspirational then we apply it
            if (currentSolution.getCost() - it->getsavings() < bestSolutionCost) {
                currentSolution.applyMove(*it);
                bestSolution = currentSolution;
                bestSolutionCost = bestSolution.getCost();
                makeTabu(*it);
                loopMadeMove = true;
                stagnationCnt = 0;
//                bestUpdatedLastAt = currentIteration;
//                ++bestUpdatedCnt;
                // update stats
//                switch (whichNeighborhood) {
//                    case Ins:     ++cntInsApplied;    break;
//                    case IntraSw: ++cntIntraSwApplied; break;
//                    case InterSw: ++cntInterSwApplied; break;
//                }
            }
        }
*/
        madeMove = madeMove or loopMadeMove;
        ++stagnationCnt;
    }
    while (madeMove and stagnationCnt < maxStagnation);

    return madeMove;
}

