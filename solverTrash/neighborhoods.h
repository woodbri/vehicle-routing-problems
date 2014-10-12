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
    void getInsNeighborhood(std::deque<Move>& moves) const ;
    void getIntraSwNeighborhood(std::deque<Move>& moves) const;
    void getInterSwNeighborhood(std::deque<Move>& moves) const;

    void v_getIntraSwNeighborhood(std::deque<Move>& moves, double factor) const;
    void v_getInsNeighborhood(std::deque<Move>& moves,double factor) ;
    void v_getInterSwNeighborhood(std::deque<Move>& moves, double factor) const;

    void applyMove(const Move&);
    bool applyInsMove( const Move &move);

};

#endif
