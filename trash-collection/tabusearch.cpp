
#include <iostream>
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
    std::cout << "   movesAdded: " << movesAdded << std::endl
              << "   movesChecked: " << movesChecked << std::endl
              << "   movesCheckedTabu: " << movesCheckedTabu << std::endl
              ;
}


bool TabuSearch::isTabu(const Move& m) {
    std::map<const Move, int>::iterator it;

    ++movesChecked;  // this makes it non-const
    for (it = TabuList.begin(); it!=TabuList.end(); ++it) {
        if (it->second < currentIteration) continue;
        if (m.tabuEquiv(it->first)) {
            ++movesCheckedTabu;  // this makes it non-const
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
    TabuList[m] = currentIteration + tabuLength;
    ++movesAdded;
}


/*
    We use a sequential solving strategy for combining our three neighborhood
    move functions. This as know as a "token-ring" search. Given an initial
    state and a set of algorithms, it makes circularly a run at each algorithm,
    always starting from the best solution found by the previous one. The
    overall process stops either when a full round of the algorithms does not
    find an improvement or the time granted is elapsed.
    
    Each single technique stops when it does not improve the current best
    solution for a given number of iterations (ie: stagnation).

*/

void TabuSearch::search() {

    currentIteration = 0;

    bool improved;
    do {
        improved = doInsMoves()
                 | doIntraSwMoves()
                 | doInterSwMoves();
    }
    while (improved and ++currentIteration < maxIteration);

}


bool TabuSearch::doInsMoves() {

}


bool TabuSearch::doIntraSwMoves() {

}


bool TabuSearch::doInterSwMoves() {

}



