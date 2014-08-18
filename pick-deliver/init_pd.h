#ifndef INIT_PD_H
#define INIT_PD_H

#include <vector>
#include <cmath>

#include "prob_pd.h"
#include "vehicle.h"

const double EPSILON = 0.001;

class Init_pd: public Prob_pd {
  public:
    //Problem P;
    //std::vector<Route> R;
    double totalDistance;
    double totalCost;

    Init_pd() {
        totalDistance = 0;
        //P=p;
        totalCost = 0;
        fleet.clear();
    };

    void dump();
    void dumproutes();
    void tau() ;
    void plotTau();
/* Diferent initial Constructions */
    void insertByOrderSolution();
    void dumbConstruction();
    void dumbConstructionAndBestMoveForward();
    void initialByOrderSolution();
    void dumbAndHillConstruction();
    void deliveryBeforePickupConstruction();
    void sequentialConstruction();
    void initialNoHillConstruction();
    void initialFeasableSolution();
    void initialConstruction();

    void computeCosts();
    double getcost();
    double getDistance();
    double getAverageRouteDurationLength();

    Init_pd& operator=( const Init_pd& rhs ) {
        if ( this != &rhs ) {
            totalDistance = rhs.totalDistance;
            totalCost = rhs.totalCost;
            fleet = rhs.fleet;
            //mapOtoR = rhs.mapOtoR;
        }
        return *this;
    };

    bool operator == (Init_pd &another) const {
        return fleeet.size() == another.fleet.size() &&
               abs(totalCost - another.totalCost) < EPSILON;
    };

   bool solutionEquivalent (Init_pd &another)  {
        computeCosts();
        another.computeCosts();
        return R.size() == another.R.size() &&
               abs(totalCost - another.totalCost) < EPSILON;

    };

    bool operator <  (Init_pd &another) const {
        return R.size() < another.R.size() || totalCost < another.totalCost;
    };
};

#endif

