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
class Solution;

const double w1 = 1.0;  // route duration weighting
const double w2 = 1000.0;  // total number of time violations weight
const double w3 = 1.0;  // total number of capacity violations weight

class Problem {
  private:
    Node depot;
  public:
    int K;      // number of vehicles
    int Q;      // capacity
    int DepotClose;
    //Node depot;
    double atwl;
    std::vector<Node> N;    // vector of nodes
    std::deque<Order> O;   // vector of orders


    // variables for plotting
    double extents[4]; 

    // Problem() {};
    // ~Problem() {};

    Node &getdepot(){ return depot;};
    void loadProblem(char *infile);

    unsigned int getNodeCount();
    bool checkIntegrity() const;

    unsigned int getOrderCount();

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
    Node& getDeliveryNodeFromOrder(int i);
    Node& getPickupNodeFromOrder(int i);
    void sortOrdersbyDist();
    void sortOrdersbyDistReverse();

    Order& getOrder(int i) ;

    void makeOrders();

    void nodesdump();
    void ordersdump();
    void dump();

    void calcAvgTWLen();
};

#endif
