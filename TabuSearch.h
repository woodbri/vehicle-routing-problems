#ifndef TABUSEARCH
#define TABUSEARCH

#include <stdexcept>
#include <limits>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Solution.h"
#include "Move.h"
#include "Tabu.h"

class TabuSearch {
  public:
    // variables for Tabu Search
    int iter;

    Solution S;
    double SCost;

    Solution Best;
    double BestCost;

    std::vector<Tabu> T;
    Move bestMove;

    int tabuLength;

    bool debugTabu;

    TabuSearch(Solution &s) : S(s), Best(s) {
        iter = 0;
        tabuLength = 30;            // set a reasonable default
        SCost = BestCost = Best.getCost();  // cost of the best
        debugTabu = false;
    };

    Solution solve();

    bool doSPI();

    bool doSBR();

    bool doWRI();

    double getAverageRouteDurationLength();

    void applyMove(Move& m);

    void addMoveTabu(Move& m);

    void addTabu(Tabu& tm);

    bool isMoveTabu(Move& m);

    bool isTabu(Tabu& tm);

    void cleanTabuList();

    void dumpTabu() {
        std::cout << "Tabu List:" << std::endl;
        for (int i=0; i<T.size(); i++) {
            std::cout << i <<":  ";
            T[i].dump();
        }
    };

};

#endif
