#ifndef TWNODE_H
#define TWNODE_H

#include <string>

#include "node.h"

class Twnode: public Node {
  protected:
    int    type;
    double demand;
    double tw_open;
    double tw_close;
    double serviceTime;

  public:
    // accessors
    double opens() const {return tw_open;};
    double closes() const {return tw_close;};
    double getdemand() const{ return demand;};
    double getservicetime() const{  return serviceTime;};
    double windowlength() const { return  tw_close - tw_open; };
    int ntype() {return type;};
    void dump() const;

    // state
    bool isvalid() const;
    bool hasdemand() const { return getdemand()>0; };
    bool hassupply() const { return getdemand()<0; };
    bool hasnogoods() const { return getdemand()==0; };
    bool earlyarrival(const double arrivalTime) const { return arrivalTime < tw_open; };
    bool latearrival(const double arrivalTime) const { return arrivalTime > tw_close; };

    // mutators
    void set(int _nid, int _id, double _x, double _y, int _demand,
             int _tw_open, int _tw_close, int _service);
    void setdemand(int _demand) { demand = _demand; };
    void settwopen(int _tw_open) { tw_open = _tw_open; };
    void settwclose(int _tw_close) { tw_close = _tw_close; };
    void setservice(int _service) { serviceTime = _service; };

    // structors

    Twnode() {
        Node();
        type=-1;
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        serviceTime = 0;
    };

    Twnode(std::string line);

    ~Twnode() {};

};
#endif
