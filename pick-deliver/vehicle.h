#ifndef VEHICLE_H
#define VEHICLE_H

#include <deque> 
#include <vector>
#include "twpath.h"
#include "order.h"
#include "dpnode.h"


class Vehicle {
  private:
    int  maxcapacity;   
    Twpath<Dpnode> path;
    Dpnode depot;
    //deque<Order> orders;

    /* for evaluating the truck */
    int  curcapacity;   
    double duration;  
    double cost;     
    int twvTot;            // number of time window violations
    int cvTot;             // number of capacity violations
    bool cv_depot;
    bool twv_depot;

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost


  public:

    // constructors

    Vehicle() {
        maxcapacity = 0;
        curcapacity = 0;
        duration    = 0;
        cost        = 0;
        twvTot         = 0;
        cvTot          = 0;
        w1 = w2 = w3 = 1.0;
    };

   Vehicle(const Dpnode &_depot,double _maxcapacity) {
        depot=_depot;
        maxcapacity=_maxcapacity;
        w1 = w2 = w3 = 1.0;
 depot.dump();
        push_back(depot);
   };

    // accessors
    int getmaxcapacity() const {return maxcapacity; };
    int size() const  {return path.size();};
 //   int  getoid(int i) const { return path[i].getoid(); };
    int getdpos(const int oid) const;
    int getppos(const int oid) const;
    std::vector<int> getpath();

    void remove(int at);
    void removeOrder(int orderid);
    void removePickup(int orderid);
    void removeDelivery(int orderid);
    void swapstops(int i,int j);
    void swap(int i,int j);
    void move(int fromi,int toj);
    void push_back(Dpnode pathstop);
    void insert(Dpnode pathstop,int at);
    //void setvalues(int at);
    //void setDepotValues();
    void dump() ;
    void smalldump();
    bool ispickup(int i) {return path[i].ispickup();}
    bool sameorder(int i,int j){return path[i].getoid()==path[j].getoid();}
    void clean() {path.resize(0); };

    /*algorithm spesific */
    void findBetterForward(int &bestI, int &bestJ);
    bool findImprovment(int i);
    void hillClimbOpt();
    int  findForwardImprovment(const int i,double &bestcost) ;





/* evaluation */
    bool feasable() { return twvTot == 0 and cvTot == 0;}
    bool hascv()const { return cvTot != 0;}
    bool hastwv()const { return twvTot != 0;}

    void evaluate();
    void evalLast();
    void evaluate(int from);
    int gettwvTot() const { return twvTot; };
    int getcvTot() const { return cvTot; };
    int getcurcapacity() const { return curcapacity; };
    double getduration() const { return duration; };
    double getcost() const { return cost; };
    //double getcost(double w1,double w2,double w3);// { return   w1*D + w2*TWV + w3*CV; }

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

    void tau() ;

    void plot(std::string file,std::string title,int carnumber);
    void pushOrder(const Order &o);
    void pushPickup(const Order &o);
    void pushDelivery(const Order &o);
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
    bool isdelivery(int i) const { return path[i].isdelivery(); };
    bool ispickup(int i) const { return path[i].ispickup(); };
    bool isdepot(int i) const { return path[i].hasnogoods(); };

};

#endif
