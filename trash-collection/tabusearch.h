#ifndef TABUSEARCH_H
#define TABUSEARCH_H

#include <map>
#include <cassert>
#include "move.h"
#include "solution.h"

class TabuSearch {

  typedef enum { Ins, IntraSw, InterSw } Neighborhoods;

  private:
    std::map<const Move, int> TabuList;

    int tabuLength;

    int maxIteration;
    int currentIteration;

    Solution bestSolution;
    double bestSolutionCost;

    Solution currentSolution;

    // keep strack of some statistics

    int movesAdded;
    int movesChecked;
    int movesCheckedTabu;

  public:
    TabuSearch(Solution initialSolution) {
        bestSolution = initialSolution;
        bestSolutionCost = bestSolution.getCost();
        currentSolution = initialSolution;
        currentIteration = 0;
        maxIteration = initialSolution.getNodeCount() * 10;
        tabuLength = std::min(initialSolution.getNodeCount() / 5, 5);
    };

    int getCurrentIteration() const { return currentIteration; };
    int getMaxIteration() const { return maxIteration; };
    int getTabuLength() const { return tabuLength; };
    Solution getBestSolution() const { return bestSolution; };
    Solution getCurrentSolution() const {return currentSolution; };
    void dumpTabuList() const;
    void dumpStats() const;
    bool isTabu(const Move& m);

    void makeTabu(const Move m);
    void cleanExpired();

    void setMaxIteration(int n) { assert(n>0); maxIteration = n; };
    void settabuLength(int n) { assert(n>0); tabuLength = n; };

    void search();
    bool doNeighborhoodMoves(Neighborhoods whichNeighborhood);

};

#endif

