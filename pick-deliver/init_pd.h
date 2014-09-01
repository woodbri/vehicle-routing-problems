#ifndef INIT_PD_H
#define INIT_PD_H

#include <vector>
#include <cmath>

#include "prob_pd.h"
#include "vehicle.h"

const double EPSILON = 0.001;

class Init_pd: public Prob_pd {
  private:
    std::deque<Vehicle> fleet;


    double totalDistance;
    double totalCost;
    double w1,w2,w3;

  public:
    Init_pd() {
        totalDistance = 0;
        //P=p;
        totalCost = 0;
        fleet.clear();
    };

    Init_pd(const Prob_pd& P):Prob_pd(P){}; 


    void setweights(double _w1,double _w2,double _w3) {w1=_w1;w2=_w2;w3=_w3;};
    void dump();
    void dumproutes();
    void tau() ;
    void plot(std::string file,std::string title);

Order getOrderData(int nodeId, int &pick, int &deliver);
bool inPending(const Vehicle pend, int nodeId) ;
int compatibleWithPending(const Vehicle &pend, int fromId, int toId); 


/* Diferent initial Constructions */
    void seqConst();
    void insertByOrderSolution();
    void dumbConstruction();
    void dumbConstructionAndBestMoveForward();
    void withSortedOrdersConstruction();
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
        }
        return *this;
    };

    bool operator == (Init_pd &another) const {
        return fleet.size() == another.fleet.size() &&
               std::abs(totalCost - another.totalCost) < EPSILON;
    };

   bool solutionEquivalent (Init_pd &another)  {
        computeCosts();
        another.computeCosts();
        return fleet.size() == another.fleet.size() &&
               std::abs(totalCost - another.totalCost) < EPSILON;

    };

    bool operator <  (Init_pd &another) const {
        return fleet.size() < another.fleet.size() || totalCost < another.totalCost;

    

    };
};

#endif

