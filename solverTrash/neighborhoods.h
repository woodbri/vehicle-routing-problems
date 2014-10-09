#ifndef NEIGHBORHOODS_H
#define NEIGHBORHOODS_H

#include "solution.h"
#include "move.h"

class Neighborhoods : public Solution {

public:
int cycles;

   //added missing Constructor
  Neighborhoods(const Solution &solution): Solution(solution){
cycles=0;
 }; 
//check for constness later
    bool isNotFeasible(const Move& m) const ;
    double getMoveSavings(const Move& m) const ;
    void getInsNeighborhood(std::vector<Move>& moves) const ;
    void getIntraSwNeighborhood(std::vector<Move>& moves) const;
    void getInterSwNeighborhood(std::vector<Move>& moves) const;

    void v_getInsNeighborhood(std::deque<Move>& moves) ;

    void applyMove(const Move&);

};

#endif
