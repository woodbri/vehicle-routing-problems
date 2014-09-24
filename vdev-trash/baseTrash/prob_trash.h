#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

//#include "order.h"
//#include "orders.h"
#include "bucketn.h"
#include "twpath.h"
#include "twc.h"

class Prob_pd {
  protected:
typedef  TwBucket<Trashnode> Bucket;

    Trashnode depot;

    TWC<Trashnode> twc;
    //Bucket datanodes;
    Twpath<Trashnode> datanodes;

//    Orders ordersList;
    std::string datafile;


  public:
    int K;      // number of vehicles
    int Q;      // capacity

    Prob_pd(char *infile);
    Trashnode getdepot() const { return depot;};
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
    Trashnode& getDeliveryNodeFromOrder(int i);
    Trashnode& getPickupNodeFromOrder(int i);
    void sortNodeById();
    void sortNodeByDistReverse();
    void sortNodeByTWC();
    void sortOrdersbyDist();
    void sortOrdersbyId();
    void sortOrdersbyIdReverse();
    void sortOrdersbyDistReverse();


    void twcijDump() const;

//    Order& getOrder(int i) ;

//    void makeOrders();

    void nodesdump();
    void plot(Plot<Trashnode> &graph);
//    void ordersdump( const std::deque<Order> orders ) const;
   void dump();


   inline double _MAX() { (std::numeric_limits<double>::max()); };
   inline double _MIN() { ( - std::numeric_limits<double>::max() ); };

    //void calcAvgTWLen();
};

#endif
