#ifndef TWNODE_H
#define TWNODE_H

#include <string>

#include "node.h"

class Twnode: public Node {
  protected:
    int demand;
    int tw_open;
    int tw_close;
    int service;

  public:
    int opens() const {return tw_open;};
    int closes() const {return tw_close;};
    int getdemand() const{ return demand;};
    int getservicetime() const{  return service;};
    int windowlength() const { return  tw_close - tw_open; };

    bool checkintegrity() const;
    bool hasdemand() const { return getdemand()>0; };
    bool hassupply() const { return getdemand()<0; };
    bool hasnogoods() const { return getdemand()==0; };
    bool earlyarrival(const double D) const { return D < tw_open; };
    bool latearrival(const double D) const { return D > tw_close; };

    void dump() const;

    void setvalues(int _nid, double _x, double _y, int _demand,
                   int _tw_open, int _tw_close, int _service);
    void setdemand(int _demand) { demand = _demand; };
    void settwopen(int _tw_open) { tw_open = _tw_open; };
    void settwclose(int _tw_close) { tw_close = _tw_close; };
    void setservice(int _service) { service = _service; };

    Twnode() {
        Node();
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        service = 0;
    };

    Twnode(std::string line);

    ~Twnode() {};

};
#endif
