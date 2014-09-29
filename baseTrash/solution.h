#ifndef SOLUTION_H
#define SOLUTION_H

#include <deque>
#include <cmath>

#include "prob_trash.h"
#include "twbucket.h"
#include "twpath.h"
#include "vehicle.h"
#include "plot.h"
//#include "orders.h"

const double EPSILON = 0.001;

class Solution: public Prob_trash {
  protected:
typedef  TwBucket<Trashnode> Bucket;

    std::deque<Vehicle> fleet;


    double totalDistance;
    double totalCost;
    double w1,w2,w3;

  public:

    Solution(const Prob_trash& P):Prob_trash(P){}; 


    void setweights(double _w1,double _w2,double _w3) {w1=_w1;w2=_w2;w3=_w3;};
    void dump();
    void dumproutes();
    void tau() ;
    void plot(std::string file,std::string title);
    std::string solutionAsText() const ;
    std::string solutionAsTextID() const ;
    std::vector<int>  solutionAsVector() const ;
    std::vector<int>  solutionAsVectorID() const ;


    void computeCosts();
    double getCost();
    double getDistance();
    double getAverageRouteDurationLength();

    Solution& operator=( const Solution& rhs ) {
        if ( this != &rhs ) {
            totalDistance = rhs.totalDistance;
            totalCost = rhs.totalCost;
            fleet = rhs.fleet;
        }
        return *this;
    };

    bool operator == (Solution &another) const {
        return fleet.size() == another.fleet.size() &&
               std::abs(totalCost - another.totalCost) < EPSILON;
    };

   bool solutionEquivalent (Solution &another)  {
        computeCosts();
        another.computeCosts();
        return fleet.size() == another.fleet.size() &&
               std::abs(totalCost - another.totalCost) < EPSILON;

    };

    bool operator <  (Solution &another) const {
        return fleet.size() < another.fleet.size() || totalCost < another.totalCost;

    

    };
};

#endif

