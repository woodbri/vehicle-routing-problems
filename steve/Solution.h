#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <cmath>

#include "Problem.h"
#include "Route.h"

const double EPSILON = 0.001;

class Solution {
  public:
    Problem& P;
    std::vector<Route> R;
    std::vector<int> mapOtoR;
    double totalDistance;
    double totalCost;

    Solution(Problem& p) : P(p) {
        totalDistance = 0;
        totalCost = 0;
        mapOtoR.clear();
        mapOtoR.resize(P.O.size());
        for (int i=0; i<P.O.size(); i++)
            mapOtoR[i] = -1;
        R.clear();
    };

    void dump();

    void sequentialConstruction();

    void initialConstruction();

    void computeCosts();

    double getCost();

    double getDistance();

    double getAverageRouteDurationLength();

    Solution& operator=( const Solution& rhs ) {
        if ( this != &rhs ) {
            totalDistance = rhs.totalDistance;
            totalCost = rhs.totalCost;
            P = rhs.P;
            R = rhs.R;
            mapOtoR = rhs.mapOtoR;
        }
        return *this;
    };

    bool operator == (Solution &another) const {
        return R.size() == another.R.size() &&
               abs(totalCost - another.totalCost) < EPSILON;
    };

    bool operator <  (Solution &another) const {
        return R.size() < another.R.size() || totalCost < another.totalCost;
    };

};

#endif

