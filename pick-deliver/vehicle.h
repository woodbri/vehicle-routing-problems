#ifndef VEHICLE_H
#define VEHICLE_H

#include <deque> 
#include "twpath.h"
#include "order.h"
#include "dpnode.h"


class Vehicle {
  private:
    int  maxcapacity;   
    Twpath<Dpnode> path;
    //deque<Order> orders;

    /* for evaluatin the truck */
    int  curcapacity;   
    double duration;  
    double cost;     
    int TWV;            // number of time window violations
    int CV;             // number of capacity violations
    bool cv_depot;
    bool twv_depot;

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost


  public:

    // structors

    Vehicle() {
        //maxcapacity = 0;
        maxcapacity = 0;
        curcapacity = 0;
        duration    = 0;
        cost        = 0;
        TWV         = 0;
        CV          = 0;
        w1 = w2 = w3 = 1.0;
    };

    // accessors
    int getmaxcapacity() const {return maxcapacity; };
    int size() const  {return path.size();};
 //   int  getoid(int i) const { return path[i].getoid(); };
    int getdpos(const int oid) const;
    int getppos(const int oid) const;

    void remove(int at);
    void removeOrder(int orderid);
    void removePickup(int orderid);
    void removeDelivery(int orderid);
    void swapnodes(int i,int j);
    void swap(int i,int j);
    void move(int fromi,int toj);
    void push_back(Dpnode pathstop);
    void insert(Dpnode pathstop,int at);
    void setvalues(int at);
    void setDepotValues();
    void dump() ;
    void smalldump();
    //bool ispickup(int i) {return path[i].ispickup();}
    //bool isdelivery(int i) {return path[i].isdelivery();}
    //bool isdepot(int i) {return path[i].isdepot();}
    bool sameorder(int i,int j){return path[i].getoid()==path[j].getoid();}
    bool feasable() { return TWV == 0 and CV == 0;}
    bool hascv()const { return CV != 0;}
    bool hastwv()const { return TWV != 0;}
    double getcost(double w1,double w2,double w3);// { return   w1*D + w2*TWV + w3*CV; }








/* evaluation */
    int getTWV() const { return TWV; };
    int getCV() const { return CV; };
    int getcurcapacity() const { return curcapacity; };
    double getduration() const { return duration; };
    double getcost() const { return cost; };
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };

    // these should be const
    double distancetodepot(int i) { return path[i].distance(path.getdepot()); };
    double distancetodump(int i) { return path[i].distance(path.getdumpsite()); };


    // mutators
    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    void evaluate();
    void tau() ;

    void plot(std::vector<double> &x, std::vector<double> &y,std::vector<int> &label,std::vector<int> &color);
    void addOrder(const Order &o);
    void addPickup(const Order &o);
    void addDelivery(const Order &o);
    void insertPickup(const Order &o, const int at);


    /*myshortcurts*/
    int getnid(int i) const { return path[i].getnid(); }
    int getoid(int i) const { return path[i].getoid(); }
    double getx(const int i) const {path[i].getx();}
    double gety(const int i) const {path[i].gety();}

    bool hasdemand(int i) const { return path[i].hasdemand(); };
    bool hassupply(int i) const { return path[i].hassupply(); };
    bool hasnogoods(int i) const { return path[i].hasnogoods(); };
    bool earlyarrival(int i,const double D) const { return path[i].earlyarrival(D); };
    bool latearrival(int i,const double D) const { return path[i].latearrival(D); };
    bool ontime(int i, const double D) const {return not earlyarrival(i,D) and not latearrival(i,D);};
    bool isdelivery(int i) const { return path[i].hasdemand(); };
    bool ispickup(int i) const { return path[i].hassupply(); };
    bool isdepot(int i) const { return path[i].hasnogoods(); };






};

/*
#endif
ifndef ROUTE_H
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

 void addNode(pathNode &node) {

    routePath.push_back(node);

}

};

*/
#endif
