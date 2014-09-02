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
    Dpnode backToDepot;
    double cost;     
    //deque<Order> orders;

    /* for evaluating the truck */
    int  curcapacity;   

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost


  public:

    // constructors

    Vehicle() {
        maxcapacity = 0;
        curcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
    };

   Vehicle(const Dpnode &_depot,double _maxcapacity) {
        backToDepot=_depot;
        maxcapacity=_maxcapacity;
        w1 = w2 = w3 = 1.0;
        push_back(_depot);
   };

    // accessors
    int getmaxcapacity() const {return maxcapacity; };
    int size() const  {return path.size();};
 //   int  getoid(int i) const { return path[i].getoid(); };
    int getpos(const int nodeId) const;
    int getdpos(const int oid) const;
    int getppos(const int oid) const;
    Twpath<Dpnode> getpath() ;
    Dpnode& operator[](unsigned int n) { return path[n]; };
    Dpnode  operator[] (unsigned int n) const { return path[n]; };


    void remove(int at);
    void removeOrder( const Order &order);
    void removeOrder(int orderid);
    void removePickup(int orderid);
    void removeDelivery(int orderid);
    void removeNode(int nodeid);
    void swapstops(int i,int j);
    void korenamaewaruidesu(Vehicle &rhs, int i, int j);
    void swap(int i,int j);
    void move(int fromi,int toj);
    void push_back(Dpnode pathstop);
    void insert(Dpnode pathstop,int at);
    int  getnid(int at) {return path[at].getnid();};
    Dpnode& getnode(int at) {return path[at];};

    void dump() const ;
    void smalldump()const ;
    bool ispickup(int i) {return path[i].ispickup();}
    bool sameorder(int i,int j){return path[i].getoid()==path[j].getoid();}
    void erase() {path.resize(0);};
    void clean() {path.resize(1,maxcapacity); evalLast(); };

    /*algorithm spesific */
    void findBetterForward(int &bestI, int &bestJ);
    bool findImprovment(int i);
    void hillClimbOpt();
    int  findForwardImprovment(const int i,double &bestcost) ;
    double costBetterPickupBackward(int &bppos, int &bdpos);
    double findBestCostBackForw(const int oid,int &bppos,int &bdpos);
    int    findBetterDeliveryForward(const int ppos,const int dpos,double &bestcost);
    bool hasTrip() {return path.size()>1;};
    bool hasNodes() {return !path.empty();};

bool isEmpty() const;
bool isEmptyTruck() const;



/* evaluation */
    bool feasable() { return backToDepot.gettwvTot() == 0 and backToDepot.getcvTot() == 0;}
    bool hascv()const { return backToDepot.getcvTot() != 0;}
    bool hastwv()const { return backToDepot.gettwvTot() != 0;}

    void evaluate();
    void evalLast();
    int gettwvTot() const { return backToDepot.gettwvTot(); };
    int getcvTot() const { return backToDepot.getcvTot(); };
    int getcurcapacity() const { return curcapacity; };
    double getduration() const { return backToDepot.gettotDist(); };
    double getcost() const { return cost; };
    //double getcost(double w1,double w2,double w3);// { return   w1*D + w2*TWV + w3*CV; }

    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };

    // these should be const
    //double distancetodepot(int i) { return path[i].distance(path.getdepot()); };
    //double distancetodump(int i) { return path[i].distance(path.getdumpsite()); };


    // mutators
    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    void tau() const ;

    void plot(std::string file,std::string title,int carnumber);
    void pushOrder(const Order &o);
    void pushPickup(const Order &o);
    void pushDelivery(const Order &o);
    void insertPickup(const Order &o, const int at);


    /* my inline functions */
    inline int getnid(int i) const { return path[i].getnid(); }
    inline int getoid(int i) const { return path[i].getoid(); }
    inline double getx(const int i) const {path[i].getx();}
    inline double gety(const int i) const {path[i].gety();}
    inline bool hasdemand(int i) const { return path[i].hasdemand(); };
    inline bool hassupply(int i) const { return path[i].hassupply(); };
    inline bool hasnogoods(int i) const { return path[i].hasnogoods(); };
    inline bool earlyarrival(int i,const double D) const { return path[i].earlyarrival(D); };
    inline bool latearrival(int i,const double D) const { return path[i].latearrival(D); };
    inline bool ontime(int i, const double D) const {return not earlyarrival(i,D) and not latearrival(i,D);};
    inline bool isdelivery(int i) const { return path[i].isdelivery(); };
    inline bool ispickup(int i) const { return path[i].ispickup(); };
    inline bool isdepot(int i) const { return path[i].hasnogoods(); };

};

typedef Vehicle Bucket;
#endif
