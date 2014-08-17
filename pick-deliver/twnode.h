#ifndef TWNODE_H
#define TWNODE_H
#include <stdexcept>
#include <math.h>

#include <iostream>
#include "node.h" 

//class Order;

class Twnode: public Node {
protected:
    int demand;     // capacity demand
    int tw_open;    // earliest window time
    int tw_close;   // latest window time
    int service;    // service time

  public:
    int opens() const {return tw_open;};
    int closes() const {return tw_close;};
    double getdemand() const{ return demand;};
    double getServiceTime() const{  return service;};
    int windowLength() const { return  tw_close - tw_open; };
/****/
    bool checkintegrity() const;
    bool earlyArrival(double D) const { return D < tw_open;};
    bool lateArrival(double D) const { return D > tw_close;};
//probalby change names
    bool hasdemand() const { return demand>0; };
    bool hassupply() const { return demand<0; };
    bool hasnogoods() const { return demand==0; };
/****/
    void dump() const;
    void debugdump() const;


    Twnode() {
        Node();
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        service = 0;
    };
    Twnode(const Twnode &tw):Node(tw){
        demand = tw.demand;
        tw_open = tw.tw_open;
        tw_close = tw.tw_close;
        service = tw.service;
    };

    Twnode(const Node &n):Node(n) {
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        service = 0;
    };
    Twnode(int nid,double x, double y):Node(nid,x,y) {
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        service = 0;
    };

    Twnode(std::string line) ;
    ~Twnode() {};

};

#endif
