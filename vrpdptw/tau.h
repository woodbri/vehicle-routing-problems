#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <cmath>

#include "problem.h"
#include "route.h"

const double EPSILON = 0.001;

class Tau {
  public:
    Problem P;
    std::vector<Route> R;
    double totalDistance;
    double totalCost;

    Tau(Problem& p): P(p) {
        totalDistance = 0;
        P=p;
        totalCost = 0;
        R.clear();
    };

    void dump();
    void dumproutes();
    void tau() ;
    void plotTau();

//    void insertByOrderSolution()
//dumbConstruction();
//    void dumbConstructionAndBestMoveForward();
    void initialByOrderSolution();
//    void dumbAndHillConstruction();
//    void deliveryBeforePickupConstruction();
//    void sequentialConstruction();
//    void initialNoHillConstruction();

    void initialFeasableSolution();
//    void initialConstruction();

    void computeCosts();

    double getcost();

    double getDistance();

    double getAverageRouteDurationLength();

    Tau& operator=( const Tau& rhs ) {
        if ( this != &rhs ) {
            totalDistance = rhs.totalDistance;
            totalCost = rhs.totalCost;
            P = rhs.P;
            R = rhs.R;
            //mapOtoR = rhs.mapOtoR;
        }
        return *this;
    };

    bool operator == (Tau &another) const {
        return R.size() == another.R.size() &&
               abs(totalCost - another.totalCost) < EPSILON;
    };

   bool solutionEquivalent (Tau &another)  {
        computeCosts();
        another.computeCosts();
        return R.size() == another.R.size() &&
               abs(totalCost - another.totalCost) < EPSILON;

    };

    bool operator <  (Tau &another) const {
        return R.size() < another.R.size() || totalCost < another.totalCost;
    };
};

#endif

