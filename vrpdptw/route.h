#ifndef ROUTE_H
#define ROUTE_H

#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

class Solution;     // forward reference

#include "order.h"
#include "pathnode.h"
#include "path.h"
#include "problem.h"

class Route {
  public:
    int rid;

    Problem& P;

    Path routePath;

    std::vector<int> path;      // node ids along the path
    std::vector<int> orders;    // order ids associated with the nodes
//    std::vector<int> capacity;  // capacity after node is loaded
//    std::vector<double> pdist;  // distance at node max(arrival time, tw_open)

    bool updated;
    //int D;      // duration
    double D;      // duration
    int TWV;    // TW violations
    int CV;     // capacity violations
    double cost;

    // these are used by testPath()
    int tD;      // duration
    int tTWV;    // TW violations
    int tCV;     // capacity violations

    Route(Problem& p);

    // ~Route() {};

    Route &operator = (const Route &r) { P = r.P; return *this; };

    int getnid(int i) const { return routePath.getnid(i); }
    int getoid(int i) const { return routePath.getoid(i); }
    double getx(const int i) const {routePath.getx(i);}
    double gety(const int i) const {routePath.gety(i);}

    int getppos (const int oid) const;
    int getdpos (const int oid) const;



    void update();
    bool earlyArrival(int pathstop,double D) const;
    bool lateArrival(int  pathstop,double D) const;
    double distanceToPrev(int pathstop);
    double distanceToNext(int pathstop);
    int nodeDemand(int pathstop) const;   
    int nodeServiceTime (int pathstop) const;   
    double testPath(const std::vector<int>& tp);
    bool capacityViolation(double q) const;


    bool insertOrder(int oid, bool mustBeValid);
    void removeOrder(const Order &o);
    void removeOrder(const int oid);

    void move(int fromi,int toj);
    int findForwardImprovment(const int i,double &bestcost);
    void findBetterForward(int &bestI, int &bestJ);
    void swapnodes(int i,int j) { routePath.swapnodes(i,j);}
    void swap(int i,int j) { routePath.swap(i,j);}
    bool ispickup(int i) {return routePath.ispickup(i);}
    bool isdelivery(int i) {return routePath.isdelivery(i);}
    bool isdepot(int i) {return routePath.isdepot(i);}
    bool sameorder(int i,int j) {return routePath.sameorder(i,j);}
    double getcost() {return routePath.getcost(w1,w2,w3);}
    double feasable() {return routePath.feasable();}
    int findBetterDeliveryForward(const int ppos,const int dpos,double &bestcost);
    double costBetterPickupBackward(int &bppos, int &bdpos) ;
    double findBestCostBackForw(const int oid,int &bppos,int &bdpos);



    void addOrder(const Order &o);
    void addPickup(const Order &o);
    void addDelivery(const Order &o);
    void insertPickup(const Order &o, const int at);
    void remove(const int at);
    bool findImprovment(int i);
    void hillClimbOpt();
//    void smalldump();
    void dumppath();
    void dump();
    void tau() ;
    void plotTau(std::vector<double> &x, std::vector<double> &y,std::vector<int> &label,std::vector<int> &color);

 void addNode(pathNode &node) {

    routePath.push_back(node);

}

};

#endif
