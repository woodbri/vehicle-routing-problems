#ifndef VEHICLE_H
#define VEHICLE_H

#include <deque> 
#include <vector>
#include "twpath.h"
//#include "order.h"
#include "trashnode.h"
#include "twc.h"
#include "bucketn.h"

class Orders;



class Vehicle:public BucketN {
  private:
typedef  TwBucket<Trashnode> Bucket;
    int vid;
    int ntype;
    double  maxcapacity; 
    Trashnode backToDepot;
    Trashnode dumpSite;

    double cost;     

    /* for evaluating the truck */
    double  curcapacity;   

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
   Vehicle(std::string line,const Bucket &depots, const Bucket &dumps, int offset ) {
       std::istringstream buffer( line );
       int depotId;
       buffer >> vid;
       buffer >> ntype;
       buffer >> depotId;
       buffer >> maxcapacity;
       backToDepot=depots[offset+depotId];
       push_back(backToDepot);
       dumpSite=dumps[0]; //Election of dumsite has change depending on other problems 
       curcapacity = 0;
       cost        = 0;
       w1 = w2 = w3 = 1.0;
   }
 
       

   Vehicle(const Trashnode &_depot,double _maxcapacity) {
        backToDepot=_depot;
        maxcapacity=_maxcapacity;
        w1 = w2 = w3 = 1.0;
        push_back(_depot);
   };

   std::deque<int> getpath() const;


    bool isvalid() const ;
    // accessors
    int getmaxcapacity() const {return maxcapacity; };
    const Trashnode& getBackToDepot() const {return backToDepot;}
    


    // return cost: infinity
    // orders are not inserted
    int   getPosLowLimit(int nid ,int from, const TWC<Trashnode> &twc ) const;
    int   getPosHighLimit(int nid ,int from, int to, const TWC<Trashnode> &twc) const;

    int pos(int nid) const ;
    bool  in(int nid) const;
    
    void e_push_back(const Trashnode &pathstop);
    void e_insert(const Trashnode &pathstop,int at);
    void e_erase(int at);
    void e_swap(int i,int j);
    void e_move(int fromi,int toj);
    void e_clean();

    void swapstops(int i,int j);

    Trashnode& getnode(int at) {return path[at];};

    void dump() const ;
    void smalldump()const ;
    void erase() {path.resize(1);};
    void clean() {path.e_resize(1,maxcapacity); evalLast(); };

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

    void evaluate(int from);
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
    void plot(Plot<Trashnode> graph,int carnumber);

};

#endif
