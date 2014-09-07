#ifndef BUCKETN_H
#define BUCKETN_H

#include <deque> 
#include <vector>
#include "twpath.h"
#include "order.h"
#include "dpnode.h"


class BucketN {
  protected:
    Twpath<Dpnode> path;

  public:

    // constructors

    BucketN() {};

    BucketN(const BucketN &truck) {
        path=truck.path;
    };
  // overload Bucket=Vehicle
    BucketN& operator=(const BucketN &truck) {
        if (this == &truck) return *this;
        path=truck.path;
        return *this;
    };
 

    // accessors
    int size() const  {return path.size();};
    int getpos(const int nodeId) const;
    int getdpos(const int oid) const;
    int getppos(const int oid) const;


    Twpath<Dpnode>&  Path() ;
    Twpath<Dpnode>  Path() const ;
    Twpath<Dpnode>  getpath() const ;
    Dpnode& operator[] (unsigned int n) { return path[n]; };
    Dpnode  operator[] (unsigned int n) const { return path[n]; };
    Dpnode  front() const {return path.front();}
    Dpnode& front() {return path.front();}


    void remove(int at);
    void removeOrder( const Order &order);
    void removeOrder(int orderid);
    void removePickup(int orderid);
    void removeDelivery(int orderid);
    void removeNode(int nodeid);
    void swapstops(int i,int j);
    void swap(int i,int j);
    void move(int fromi,int toj);
    void push_back(Dpnode pathstop);
    void insert(const Dpnode &pathstop,int at);
    void pushOrder(const Order &o);
    void pushPickup(const Order &o);
    void pushDelivery(const Order &o);
    void insertPickup(const Order &o, const int at);


    Dpnode& getnode(int at) {return path[at];};

    void dump() const ;
    void smalldump()const ;
    void tau() const ;
    bool sameorder(int i,int j){return path[i].getoid()==path[j].getoid();}
    void erase() {path.resize(0);};

    /*algorithm spesific */
    bool hasNodes() {return !path.empty();};

    bool empty() const;




    void plot(std::string file,std::string title,int carnumber);


    /* my inline functions */
    //int  getnid(int at) {return path[at].getnid();};
    bool in(int nid);
    int  pos(int nid);
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


double distanceToPrev(int pathstop) {
     if (pathstop == 0)  return 0; // distance from depot to depot is allways 0
     else  return path[pathstop].distance(path[pathstop-1]);
}

double distanceToNext(int pathstop) {
     if (pathstop == path.size()-1)  return 0;
     else  return path[pathstop].distance(path[pathstop+1]);
}


};

#endif
