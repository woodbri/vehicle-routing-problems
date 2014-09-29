#ifndef BUCKETN_H
#define BUCKETN_H

#include <iostream>
#include <sstream>
#include <deque> 
#include "twpath.h"
#include "trashnode.h"
#include "plot.h"



class BucketN {
  protected:
    Twpath<Trashnode> path;

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


    Twpath<Trashnode>&  Path() ;
    const Twpath<Trashnode>&  Path() const ;
    const Twpath<Trashnode>&  getpath() const ;

    Trashnode& operator[] (unsigned int n) { return path[n]; };
    const Trashnode&  operator[] (unsigned int n) const { return path[n]; };

    int size() const  {return path.size();};
    void resize(int s) {return path.resize(s);};
    Trashnode  front() const {return path.front();}
    Trashnode& front() {return path.front();}
    


    void remove(int at);
    void erase(int at) {remove(at);};
    void removeNode(int nid);
    void swapstops(int i,int j);
    void swap(int i,int j);
    void move(int fromi,int toj);
    void push_back(Trashnode pathstop);
    void insert(const Trashnode &pathstop,int at);

    Trashnode& getnode(int at) {return path[at];};

    void dump() const ;
    void smalldump()const ;
    void tau() const ;
    void erase() {path.resize(0);};

    /*algorithm spesific */
    bool hasNodes() {return !path.empty();};
    bool empty() const;




    void plot(std::string file,std::string title,int carnumber);


    /* my inline functions */
    //int  getnid(int at) {return path[at].getnid();};
    bool in(int nid) const;
    bool in(const Trashnode &node) const;
    int  pos(int nid) const;
    int  pos(const Trashnode &node) const;
    inline int getnid(int pos) const { return path[pos].getnid(); }
    inline int getid(int pos) const { return path[pos].getid(); }
    inline double getx(const int pos) const {path[pos].getx();}
    inline double gety(const int pos) const {path[pos].gety();}
    inline bool hasdemand(int pos) const { return path[pos].hasdemand(); };
    inline bool hassupply(int pos) const { return path[pos].hassupply(); };
    inline bool hasnogoods(int pos) const { return path[pos].hasnogoods(); };
    inline bool earlyarrival(int pos,const double D) const { return path[pos].earlyarrival(D); };
    inline bool latearrival(int pos,const double D) const { return path[pos].latearrival(D); };
    inline bool ontime(int pos, const double D) const {return not earlyarrival(pos,D) and not latearrival(pos,D);};
    inline bool isdump(int pos) const { return path[pos].isdump(); };
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
