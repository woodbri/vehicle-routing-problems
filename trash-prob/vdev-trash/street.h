#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <limits>
#include <sstream>


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "plot.h"


class Street    {
  private:
typedef  TwBucket<Trashnode> Bucket;
typedef  unsigned long int UID ;
    int sid;
    Twpath<Trashnode> path;

    int _reqCapacity;
    int _reqTime;

  inline double MAX() { (std::numeric_limits<double>::max()); };

  protected:


  public:

    //--------------------------------------------------------------------
    // structors
    //--------------------------------------------------------------------

    Street() {
       sid=-1;
    };

    Street( const Trashnode &node) {
        sid=node.streetId();
        path.push_back( node );
        path.evalLast(MAX());
    }


    //--------------------------------------------------------------------
    // accessors
    //--------------------------------------------------------------------

    double reqCapacity();
    double reqTime();

    int size() const { return path.size(); };
    int streetId() const { return sid; };

    //--------------------------------------------------------------------
    // dumps and plots
    //--------------------------------------------------------------------
    void dump() const;
    void dump(const std::string &title) const;
    void dumpeval() const;
    void smalldump() const;
    void dumppath() const;
    void tau() const ;

    void plot(std::string file,std::string title);
    void plot(Plot<Trashnode> graph);


    void evalLast();


    //--------------------------------------------------------------------
    // wrappers to twpath code to 
    //--------------------------------------------------------------------
/*
    bool push_back(Trashnode node);
    bool push_front(Trashnode node);
    bool insert(Trashnode node, int at);
    bool remove( int at);
    bool moverange( int rangefrom, int rangeto, int destbefore );
    bool movereverse( int rangefrom, int rangeto, int destbefore );
    bool reverse( int rangefrom, int rangeto );
    bool move( int fromi, int toj );
    bool swap( const int& i, const int& j );

*/

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
    bool ispickup(int i) const { return path[i].ispickup(); };

    Trashnode operator[](int i) const { return path[i]; };

};


#endif

