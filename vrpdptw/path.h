#ifndef PATH_H
#define PATH_H

//#include <stdexcept>
//#include <algorithm>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
#include <deque>
//#include <math.h>

//#include "order.h"
//#include "node.h"
#include "dpnode.h"
//#include "problem.h"  
//eventually problem.h wont be added herethis disapears


class Path{
  private:
    Twnode depot;
//    double w1,w2,w3;
    int rid;
    double D;      // duration
    int TWV;    // TW violations
    int CV;     // capacity violations
    bool twv_depot;
    bool cv_depot;
    std::deque<pathNode> path;      // node ids along the path and more
       
  public:



    Path (const pathNode &d){
         depot = d;
         twv_depot=false;
         cv_depot=false;
	 D= TWV=CV=0;
//         w1=w2=w3=1;
         pathNode dep(d);
         path.push_back(dep);
     };

    ~Path() {};

    int size() {return path.size();};
    double getx(const int i) const { return path[i].getx(); };
    double gety(const int i) const { return path[i].gety(); };
    int getnid(int i) const { return path[i].getnid(); };
 //   int  getoid(int i) const { return path[i].getoid(); };
    int getdpos(const int oid) const;
    int getppos(const int oid) const;
    void remove(int at);
    void removeOrder(int orderid);
    void removePickup(int orderid);
    void removeDelivery(int orderid);
    void swapnodes(int i,int j);
    void swap(int i,int j);
    void move(int fromi,int toj);
    void push_back(pathNode pathstop);
    void insert(pathNode pathstop,int at);
    void setvalues(int at);
    void setDepotValues();
    void dump() ;
    void smalldump();
    bool ispickup(int i) {return path[i].ispickup();}
    bool isdelivery(int i) {return path[i].isdelivery();}
    bool isdepot(int i) {return path[i].isdepot();}
//    bool sameorder(int i,int j){return path[i].sameorder(path[j]);}
    bool feasable() { return TWV == 0 and CV == 0;}
    bool hascv()const { return CV != 0;}
    bool hastwv()const { return TWV != 0;}
    double getcost(double w1,double w2,double w3);// { return   w1*D + w2*TWV + w3*CV; }


};
#endif
