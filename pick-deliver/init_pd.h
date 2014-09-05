#ifndef INIT_PD_H
#define INIT_PD_H

#include <deque>
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

/* OrderIncompatibility initial Solution */
double compatibleIJ(int fromNid, int toNid)const; 
void sortOrdersByIncompat(std::deque<Order> orders);
bool incompatible(int fromId, int toId) const ;
bool isIncompatibleOrder(const Order &orderx, const Order &ordery) const; 
bool isIncompatibleOrder(int oidx,int oidy ) const ;
bool isCompatibleOrder(const Order &from, const Order &to) const ;
bool isCompatibleOrder(int oidx,int oidy) const ;
double compatOrdersMeasure(const Order &orderx, const Order &ordery) const;
int countCompatibleOrders(const Order &from, const std::deque<Order> &to) const;
int getMostCompatibleOrder(const std::deque<Order> &orders) ;
void removeIncompatibleOrders(const Order &from,  std::deque<Order> &orders, std::deque<Order> &incompatible) ;
void insertInTruck(Vehicle &truck,const Order &order);
void makeRoute(Vehicle &truck, std::deque<Order> &orders, std::deque<Order> &incompatible);
    void orderConstraintConstruction();



/*  paper initial Construction */
void insert(Vehicle &truck, int nodeId,Bucket &nodes);
void removeIncompatible(int fromId,Bucket &nodes, Bucket &incomatible);
void paperConst(Vehicle&, Bucket&, Bucket&, Bucket&);
bool isCompatibleWithPending(int fromId,const Bucket &pending) ;

    void seqConst();

/* Diferent initial Constructions */
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

