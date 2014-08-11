#ifndef TWNODE_H
#define TWNODE_H

#include <string>

#include "node.h"

class Twnode: public Node {
  private:
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
    bool ispickup() const { return getdemand()>0; };
    bool isdelivery() const { return getdemand()<0; };
    bool earlyarrival(const double D) const { return D < tw_open; };
    bool latearrival(const double D) const { return D > tw_close; };

    void dump() const;

    void setvalues(int nid, double x, double y, int demand,
                   int tw_open, int tw_close, int service);
    void setdemand(int v) { demand = v; };
    void settwopen(int v) { tw_open = v; };
    void settwclose(int v) { tw_close = v; };
    void setservice(int v) { service = v; };

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
