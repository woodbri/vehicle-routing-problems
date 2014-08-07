#ifndef NODE_H
#define NODE_H

#include <math.h>


class Node {
  private:
    int nid;        // node number
    int ntype;      // node type (0=depot, 1=dump, 2=pickup)
    double x;       // x location
    double y;       // y location
    int demand;     // capacity demand
    int tw_open;    // earliest window time
    int tw_close;   // latest window time
    int service;    // service time
    double vehicledist;     // distance to nearest depot
    int vehiclenid;         // nid of the closet depot
    double dumpdist;        // distance to nearest dump
    int dumpnid;            // nid of closet dump

  public:
    // accessors
    int getnid() const {return nid;};
    int getvehicledist() const {return vehicledist;};
    int getvehiclenid() const {return vehiclenid;};
    int getdumpdist() const {return dumpdist;};
    int getdumpnid() const {return dumpnid;};
    int getx() const {return x;};
    int gety() const {return y;};
    int opens() const {return tw_open;};
    int closes() const {return tw_close;};
    double getDemand() const{ return demand;};
    double getServiceTime() const{  return service;};
    int windowLength() const { return  tw_close - tw_open; };

    // mutators
    void setvehicledist(int nid, double dist);
    void setdumpdist(int nid, double dist);

    // other
    bool checkIntegrity();
    bool earlyArrival(double D) const { return D < tw_open;};
    bool lateArrival(double D) const { return D > tw_close;};
    bool isvehicle() const {return ntype==0;};
    bool isdump() const {return ntype==1;};
    bool ispickup() const {return ntype==2;};
    double distance(const Node &n2) const {
        double dx = n2.x - x;
        double dy = n2.y - y;
        return sqrt( dx*dx + dy*dy );
    };

    void dump() const;

    Node() {
        nid = -1;
        ntype = -1;
        x = 0.0;
        y = 0.0;
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        service = 0;
        vehicledist = 0.0;
        vehiclenid = -1;
        dumpdist = 0.0;
        dumpnid = -1;
    };
    Node(std::string line);
    ~Node() {};

};

#endif
