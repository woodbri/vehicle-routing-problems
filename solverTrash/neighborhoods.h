#ifndef NEIGHBORHOODS_H
#define NEIGHBORHOODS_H

#include "solution.h"
#include "move.h"

class Neighborhoods : public Solution {

public:
//check for constness later
    bool isNotFeasible(const Move& m) ;
    double getMoveSavings(const Move& m) ;
    void getInsNeighborhood(std::vector<Move>& moves) ;
    void getIntraSwNeighborhood(std::vector<Move>& moves) ;
    void getInterSwNeighborhood(std::vector<Move>& moves) ;

    void applyMove(const Move&);

};

#endif
