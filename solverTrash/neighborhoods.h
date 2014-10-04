#ifndef NEIGHBORHOODS_H
#define NEIGHBORHOODS_H

#include "solution.h"
#include "move.h"

class Neighborhoods : public Solution {

    bool isNotFeasible(const Move& m) const;
    double getMoveSavings(const Move& m) const;
    void getInsNeighborhood(std::vector<Move>& moves) const;
    void getIntraSwNeighborhood(std::vector<Move>& moves) const;
    void getInterSwNeighborhood(std::vector<Move>& moves) const;

    void applyMove(const Move&);

};

#endif
