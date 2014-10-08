#ifndef TABUSEARCH_H
#define TABUSEARCH_H

#include <map>
#include <cassert>
#include "move.h"
#include "neighborhoods.h"

class TabuSearch {

   typedef enum { Ins, IntraSw, InterSw } neighborMovesName;

  private:
    std::map<const Move, int> TabuList;

    int tabuLength;

    int maxIteration;
    int currentIteration;

    Neighborhoods bestSolution;
    double bestSolutionCost;

    Neighborhoods currentSolution;

    // keep strack of some statistics

    int movesAdded;
    int movesChecked;
    int movesCheckedTabu;
    int bestUpdatedLastAt;
    int bestUpdatedCnt;
    int cntInsApplied;
    int cntIntraSwApplied;
    int cntInterSwApplied;

  public:
    TabuSearch(const Neighborhoods &initialSolution) :
        bestSolution(initialSolution), currentSolution(initialSolution)
    {
        bestSolution.computeCosts();
bestSolution.dump();
        bestSolutionCost = bestSolution.getCost();
        currentIteration = 0;
        maxIteration = 1000;
        tabuLength = std::min(initialSolution.getNodeCount() / 5, (unsigned int) 5);
        bestUpdatedLastAt = 0;
        bestUpdatedCnt = 0;
        cntInsApplied = cntIntraSwApplied = cntInterSwApplied = 0;
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
    bool doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxStagnation);

};

#endif

