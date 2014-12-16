#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include "move.h"

class Solution {

  public:
    double getCost() const {  /* TODO */ return 0.0; };
    int getNodeCount() const {  /* TODO */ return 50; };
    void getInsNeighborhood(std::vector<Move>& moves) { /* TODO */ };
    void getIntraSwNeighborhood(std::vector<Move>& moves) { /* TODO */ };
    void getInterSwNeighborhood(std::vector<Move>& moves) { /* TODO */ };
    void applyMove(const Move&) { /* TODO */ };
};

#endif
