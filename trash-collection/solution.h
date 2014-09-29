#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include "move.h"

class Solution {

  public:
    double getCost() const {  /* TODO */ return 0.0; };
    int getNodeCount() const {  /* TODO */ return 50; };
    std::vector<const Move> getInsNeighborhood() {
        /* TODO */
        std::vector<const Move> moves;
        return moves;
    };
    void applyMove(const Move&) { /* TODO */ };
};

#endif
