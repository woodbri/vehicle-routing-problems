#ifndef INIT_PD_H
#define INIT_PD_H

#include <deque>
#include <cmath>

#include "order.h"
#include "prob_pd.h"
#include "solution.h"
#include "vehicle.h"
#include "bucketn.h"
#include "plot.h"

//const double EPSILON = 0.001;

class Init_pd: public Solution {

  public:
/*    Init_pd() {
        totalDistance = 0;
        //P=p;
        totalCost = 0;
        fleet.clear();
    };
*/
    Init_pd(const Solution& P):Solution(P){}; 



Order getOrderData(int nodeId, int &pick, int &deliver);
bool inPending(const BucketN &pend, int nodeId) ;
int compatibleWithPending(const Vehicle &pend, int fromId, int toId); 

/* OrderIncompatibility initial Solution */
double compatibleIJ(int fromNid, int toNid)const; 
double compatibleIJ(const Order &order) const ;
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
int getBestOrder(Vehicle &truck,const std::deque<Order> &orders) const;
void makeRoute(Vehicle &truck, std::deque<Order> &orders, std::deque<Order> &incompatible);
    void orderConstraintConstruction();



/*  paper initial Construction */
void insert(Vehicle &truck, int nodeId,BucketN &nodes);
void removeIncompatible(int fromId,BucketN &nodes, BucketN &incomatible);
void paperConst(Vehicle &truck, BucketN &, BucketN&, BucketN&);
bool isCompatibleWithPending(int fromId,const BucketN &pending) ;

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

};
#endif

