#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <sstream>


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "plot.h"


class Vehicle  {
  private:
typedef  TwBucket<Trashnode> Bucket;
typedef  unsigned long int UID ;
    int vid;
    int ntype;
    Twpath<Trashnode> path;
    Trashnode backToDepot;
    Trashnode dumpsite;

    int maxcapacity;
    double cost;        // cost of the route

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost

  protected:

    // this is used when we save a copy of the path so we can make
    // changes and to restore the original path if the changes
    // do not improve the path.
    // There is a hidden assumption that path[0] == backToDepot node.

    void setvpath(Twpath<Trashnode> p) { path = p; };

  public:

    //--------------------------------------------------------------------
    // structors
    //--------------------------------------------------------------------

    Vehicle() {
        maxcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
    };

    Vehicle( Trashnode _depot, Trashnode _dump ) {
        maxcapacity  = _depot.getdemand();
        _depot.setdemand(0);
        _depot.setservice(0);
        backToDepot  = _depot;
        dumpsite = _dump;
        push_back( _depot );
        cost         = 0;
        w1 = w2 = w3 = 1.0;
    }


    Vehicle(std::string line,const Bucket &depots, const Bucket &dumps, int offset ) {
       std::istringstream buffer( line );
       int depotId;
       buffer >> vid;
       buffer >> ntype;
       buffer >> depotId;
       buffer >> maxcapacity;
       backToDepot=depots[offset+depotId];
       push_back(backToDepot);
       dumpsite=dumps[0]; //Election of dumsite has change depending on other problems
       //curcapacity = 0;
       cost        = 0;
       w1 = w2 = w3 = 1.0;
       evalLast();
   }

   bool isvalid() const {return true;};  //TBD
    bool findNearestNodeTo(Bucket &unassigned,const TWC<Trashnode> &twc,  UID &pos,  Trashnode &bestNode);

    //--------------------------------------------------------------------
    // accessors
    //--------------------------------------------------------------------

    Twpath<Trashnode> getvpath() const { return path; };
    Twpath<Trashnode>& getvpath() { return path; };
    std::deque<int> getpath() const;
    int size() const { return path.size(); };
    int getmaxcapacity() const { return maxcapacity; };
    int getTWV() const { return backToDepot.gettwvTot(); };
    int getCV() const { return backToDepot.getcvTot(); };
    int getcargo() const { return path.back().getcargo(); };
    double getduration() const { return backToDepot.gettotDist(); };
    double getcost() const { return cost; };
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };
    int getVid() const { return vid; };
    Trashnode getdepot() const { return backToDepot; };
    Trashnode& getdepot() { return backToDepot; };
    const Trashnode& getBackToDepot() const {return backToDepot;}
    Trashnode getdumpsite() const { return dumpsite; };
    Trashnode& getdumpsite() { return dumpsite; };

    double distancetodepot(int i) const { return path[i].distance(getdepot()); };
    double distancetodump(int i) const { return path[i].distance(getdumpsite()); };

    Trashnode operator[](int i) const { return path[i]; };

    //--------------------------------------------------------------------
    // dumps and plots
    //--------------------------------------------------------------------
    void dump() const;
    void dump(const std::string &title) const;
    void dumpeval() const;
    void smalldump() const;
    void dumppath() const;
    void tau() const ;

    void plot(std::string file,std::string title,int carnumber);
    void plot(Plot<Trashnode> graph,int carnumber);


    //--------------------------------------------------------------------
    // evaluation
    //--------------------------------------------------------------------

    bool feasable() const { return backToDepot.gettwvTot() == 0 and backToDepot.getcvTot() == 0; };
    bool hascv()const { return backToDepot.getcvTot() != 0; };
    bool hastwv()const { return backToDepot.gettwvTot() != 0; };

    void evalLast();

    //--------------------------------------------------------------------
    // mutators 
    //--------------------------------------------------------------------

    // these two do not work with autoeval
    // instead use Vehicle(depot, dump) constructor
    //void setdepot(Trashnode _depot) { backToDepot = _depot; };
    //void setdumpsite(Trashnode _dump) { dumpsite = _dump; };

    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    //--------------------------------------------------------------------
    // wrappers to twpath code to 
    //--------------------------------------------------------------------

    // single path manipulators

    bool push_back(Trashnode node);
    bool push_front(Trashnode node);
    bool insert(Trashnode node, int at);
    bool remove( int at);
    bool moverange( int rangefrom, int rangeto, int destbefore );
    bool movereverse( int rangefrom, int rangeto, int destbefore );
    bool reverse( int rangefrom, int rangeto );
    bool move( int fromi, int toj );
    bool swap( const int& i, const int& j );

    // multiple path manipulators

    bool swap(Vehicle& v2, const int& i1, const int& i2);

    // restore a saved path to undo an operation

    void restorePath(Twpath<Trashnode> oldpath); //tmp=v; v.dostuff; v=tmp restores as original

    //--------------------------------------------------------------------
    // algorithm specific - intra-route manipulations
    //--------------------------------------------------------------------

    bool doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4);
    bool doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6);
    bool doOrOpt(const int& c1, const int& c2, const int &c3);
    bool doNodeMove(const int& i, const int& j);
    bool doNodeSwap(const int& i, const int& j);
    bool doInvertSeq(const int& i, const int& j);

    bool pathOptimize();
    bool pathTwoOpt();
    bool pathThreeOpt();
    bool pathOrOpt();
    bool pathOptMoveNodes();
    bool pathOptExchangeNodes();
    bool pathOptInvertSequence();

    bool findBestFit(Trashnode& tn, int* tpos, double* deltacost);

    //--------------------------------------------------------------------
    // algorithm specific - inter-route manipulations
    //--------------------------------------------------------------------

    bool swap2(Vehicle& v2, const int& i1, const int& i2, bool force);
    bool swap3(Vehicle& v2, Vehicle& v3, const int& i1, const int& i2, const int& i3, bool force);
    bool exchangeSeq(Vehicle& v2, const int& i1, const int& j1, const int& i2, const int& j2, bool force);
    bool exchangeTails(Vehicle& v2, const int& i1, const int& i2, bool force);
    bool exchange3(Vehicle& v2, Vehicle& v3, const int& cnt, const int& i1, const int& i2, const int& i3, bool force);
    bool relocate(Vehicle& v2, const int& i1, const int& i2, bool force);
    bool relocateBest(Vehicle& v2, const int& i1);

    //----------------------------------------------------------------
    // I really hate these shortcuts & I love them but I'll think about them really hard
    //----------------------------------------------------------------

    int getnid(int i) const { return path[i].getnid(); };
    int getid(int i) const { return path[i].getid(); };
    double getx(const int i) const { path[i].getx(); };
    double gety(const int i) const { path[i].gety(); };
    bool hasdemand(int i) const { return path[i].hasdemand(); };
    bool hassupply(int i) const { return path[i].hassupply(); };
    bool hasnogoods(int i) const { return path[i].hasnogoods(); };
    bool earlyarrival(int i, const double D) const { return path[i].earlyarrival(D); };
    bool latearrival(int i, const double D) const { return path[i].latearrival(D); };
    bool ontime(int i, const double D) const { return not earlyarrival(i, D) and not latearrival(i, D); };
    bool isdump(int i) const { return path[i].isdump(); };
    bool ispickup(int i) const { return path[i].ispickup(); };
    bool isdepot(int i) const { return path[i].isdepot(); };


};


#endif

