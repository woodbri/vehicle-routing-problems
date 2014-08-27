#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

#include "node.h"
#include "order.h"
#include "vehicle.h"

class Prob_pd {
  protected:
    Dpnode depot;
    double w1,w2,w3;

    Twpath<Dpnode> datanodes;
    Twpath<Dpnode> originalnodes;
    std::deque<Order> ordersList;   // vector of orders
    std::deque<std::deque<double> > twcij;
    std::deque<double>  twcTot;

//    std::vector<int> unassigned;

  public:
    int K;      // number of vehicles
    int Q;      // capacity

    Dpnode getdepot() const { return depot;};
    void loadProblem(char *infile);

    unsigned int getNodeCount() const;
    bool checkIntegrity() const;

    unsigned int getOrderCount() const;

    double distance(int n1, int n2) const;
    double DepotToPickup(int n1) const ;
    double DepotToDelivery(int n1) const ;
    int getOrderOid(int i) const;
    int getOrderPid(int i) const;
    int getOrderDid(int i) const;
    double nodeDemand(int i) const;
    double nodeServiceTime(int i) const;
    bool earlyArrival(int nid,double D) const; 
    bool lateArrival(int nid,double D) const; 
    bool isAsignedOrder(int oid) const;
    Dpnode& getDeliveryNodeFromOrder(int i);
    Dpnode& getPickupNodeFromOrder(int i);
    void sortNodeByDistReverse();
    void sortNodeByTWC();
    void sortOrdersbyDist();
    void sortOrdersbyId();
    void sortOrdersbyIdReverse();
    void sortOrdersbyDistReverse();


/* compatability issues */
    void twcij_calculate();
    void twcTot_calculate();
    double ajli(const Dpnode &ni, const Dpnode &nj);
    double ajei(const Dpnode &ni, const Dpnode &nj);
    double twc_for_ij(const Dpnode &ni, const Dpnode &nj);
    double compat(int i,int j) const ;
    bool compatibleIJ(int i, int j);
    bool compatibleIAJ(int i, int a, int j);
    void dumpCompatible() ;


    void twcijDump() const;

    Order& getOrder(int i) ;

    void makeOrders();

    void nodesdump();
    void ordersdump();
    void dump();

    //void calcAvgTWLen();
};

#endif
