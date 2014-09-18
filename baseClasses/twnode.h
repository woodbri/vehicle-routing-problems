#ifndef TWNODE_H
#define TWNODE_H

#include <string>

#include "node.h"

class Twnode: public Node {
  protected:
    double demand;
    double tw_open;
    double tw_close;
    double service;

  public:
    // accessors
    double opens() const {return tw_open;};
    double closes() const {return tw_close;};
    double getdemand() const{ return demand;};
    double getservicetime() const{  return service;};
    double windowlength() const { return  tw_close - tw_open; };
    void dump() const;

    // state
    bool checkintegrity() const;
    bool hasdemand() const { return getdemand()>0; };
    bool hassupply() const { return getdemand()<0; };
    bool hasnogoods() const { return getdemand()==0; };
    bool earlyarrival(const double D) const { return D < tw_open; };
    bool latearrival(const double D) const { return D > tw_close; };

    // mutators
    void set(int _nid, int _id, double _x, double _y, int _demand,
             int _tw_open, int _tw_close, int _service);
    void setdemand(int _demand) { demand = _demand; };
    void settwopen(int _tw_open) { tw_open = _tw_open; };
    void settwclose(int _tw_close) { tw_close = _tw_close; };
    void setservice(int _service) { service = _service; };

    // structors

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

    Twnode(int nid, double x, double y):Node(nid,x,y) {
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        service = 0;
    };

    Twnode(int nid, int id, double x, double y, int _demand, int _tw_open, int _tw_close, int _service) : Node(nid,id,x,y) {
        demand = _demand;
        tw_open = _tw_open;
        tw_close = _tw_close;
        service = _service;
    };

    Twnode(std::string line);

    ~Twnode() {};

};
#endif
