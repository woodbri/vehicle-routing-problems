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
//class Solution;

//const double w1 = 1.0;  // route duration weighting
//const double w2 = 1000.0;  // total number of time violations weight
//const double w3 = 1.0;  // total number of capacity violations weight

class Prob_pd {
  protected:
    Dpnode depot;
    double w1,w2,w3;

    std::vector<Vehicle> fleet;
    std::vector<Dpnode> datanodes;
    std::vector<int> depots;
    std::vector<int> dumps;
    std::vector<int> pickups;
    std::deque<Order> ordersList;   // vector of orders

    std::vector<int> unassigned;

  public:
    int K;      // number of vehicles
    int Q;      // capacity
    int DepotClose;
    //Node depot;
    double atwl;
    std::vector<Node> N;    // vector of nodes


    // variables for plotting
    //double extents[4]; 

    // Problem() {};
    // ~Problem() {};

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
    void sortOrdersbyDist();
    void sortOrdersbyDistReverse();

    Order& getOrder(int i) ;

    void makeOrders();

    void nodesdump();
    void ordersdump();
    void dump();

    //void calcAvgTWLen();
};

#endif
