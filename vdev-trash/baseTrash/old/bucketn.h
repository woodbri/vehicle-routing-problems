#ifndef BUCKETN_H
#define BUCKETN_H

#include <iostream>
#include <sstream>
#include <deque> 
#include "twpath.h"
#include "order.h"
#include "dpnode.h"
#include "plot.h"



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


    Twpath<Dpnode>&  Path() ;
    const Twpath<Dpnode>&  Path() const ;
    const Twpath<Dpnode>&  getpath() const ;

    Dpnode& operator[] (unsigned int n) { return path[n]; };
    const Dpnode&  operator[] (unsigned int n) const { return path[n]; };

    int size() const  {return path.size();};
    void resize(int s) {return path.resize(s);};
    Dpnode  front() const {return path.front();}
    Dpnode& front() {return path.front();}
    


    void remove(int at);
    void erase(int at) {remove(at);};
    void removeOrder( const Order &order);
    void removeOrder(int oid);
    void removePickup(int oid);
    void removeDelivery(int oid);
    void removeNode(int nid);
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
    bool in(int nid) const;
    bool in(const Dpnode &node) const;
    bool in(const Order &o) const;
    int  pos(int nid) const;
    int  pos(const Dpnode &node) const;
    int  getdpos(int oid) const;
    int  getppos(int oid) const;
    int  getdpos(const Order &o) const;
    int  getppos(const Order &o) const;

   /* based on position in the path*/ 
    inline int getnid(int pos) const { return path[pos].getnid(); }
    inline int getoid(int pos) const { return path[pos].getoid(); }
    inline double getx(const int pos) const {path[pos].getx();}
    inline double gety(const int pos) const {path[pos].gety();}
    inline bool hasdemand(int pos) const { return path[pos].hasdemand(); };
    inline bool hassupply(int pos) const { return path[pos].hassupply(); };
    inline bool hasnogoods(int pos) const { return path[pos].hasnogoods(); };
    inline bool earlyarrival(int pos,const double D) const { return path[pos].earlyarrival(D); };
    inline bool latearrival(int pos,const double D) const { return path[pos].latearrival(D); };
    inline bool ontime(int pos, const double D) const {return not earlyarrival(pos,D) and not latearrival(pos,D);};
    inline bool isdelivery(int pos) const { return path[pos].isdelivery(); };
    inline bool ispickup(int pos) const { return path[pos].ispickup(); };
    inline bool isdepot(int pos) const { return path[pos].hasnogoods(); };


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
