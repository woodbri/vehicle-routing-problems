/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#ifndef BASEVEHICLE_H
#define BASEVEHICLE_H

#include <limits>
#include <vector>
#include <sstream>


#ifdef DOPLOT
#include "plot.h"
#endif


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "move.h"
#include "pg_types_vrp.h"


class BaseVehicle  {
  protected:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID ;
    typedef  unsigned long int POS ;
    typedef  unsigned long int UINT ;
    inline double _MAX() const { return ( std::numeric_limits<double>::max() ); };
    inline double _MIN() const { return ( - std::numeric_limits<double>::max() ); };


    int vid;
    //int ntype;
    Twpath<Trashnode> path;
    Trashnode depot; //just for keeps
    Trashnode endingSite;
    Trashnode dumpSite;
    double startTime;
    double endTime;

    double maxcapacity;
    double cost;        // cost of the route

    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost

  protected:

    // this is used when we save a copy of the path so we can make
    // changes and to restore the original path if the changes
    // do not improve the path.
    // There is a hidden assumption that path[0] == endingSite node.

    void setvpath( Twpath<Trashnode> p ) { path = p; };

  public:

    bool isvalid() const {return vid >= 0;}; //
    bool findNearestNodeTo( Bucket &unassigned,  UID &pos,  Trashnode &bestNode );
    bool e_setPath( const Bucket &sol );

    //--------------------------------------------------------------------
    // constructors
    //--------------------------------------------------------------------

    BaseVehicle();
    BaseVehicle( int _vid, int _start_id, int _dump_id, int _end_id,
                 int _capacity, int _dumpservicetime, int _starttime,
                 int _endtime, const Bucket &otherlocs );
    BaseVehicle( std::string line, const Bucket &otherlocs );


    //--------------------------------------------------------------------
    // accessors
    //--------------------------------------------------------------------

    Twpath<Trashnode> getvpath() const { return path; };
    Twpath<Trashnode> &getvpath() { return path; };
    std::deque<int> getpath() const;
    UINT size() const { return path.size(); };
    double getmaxcapacity() const { return maxcapacity; };
    int getTWV() const { return endingSite.twvTot(); };
    int getCV() const { return endingSite.cvTot(); };
    double getCargo() const { return  path[path.size() - 1].cargo(); };
    double getDuration() const { return ( path.size() - 1 == 0 ) ? 0.0 : endingSite.getTotTime(); };
    double getcost() const { return cost; };
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };
    int getVid() const { return vid; };
    inline double getCurrentCapacity() const {return (maxcapacity - path[size() - 1].cargo());}
    const Trashnode &getDepot() const { return path[0]; };
    const Trashnode &getStartingSite() const {return path[0];}
    const Trashnode &getDumpSite() const { return dumpSite; };
    const Trashnode &getEndingSite() const {return endingSite;}
    Trashnode &getDepot() { return path[0]; };
    Trashnode &getStartingSite() {return path[0];}
    Trashnode &getDumpSite()  { return dumpSite; };
    Trashnode &getEndingSite()  {return endingSite;}

#if 0
    const Trashnode& getBackToDepot() const {return endingSite;}

    double distancetodepot(int i) const { return path[i].distance(getdepot()); };
    double distancetodump(int i) const { return path[i].distance(getdumpSite()); };

    #ifdef WITHOSRM
    //--------------------------------------------------------------------
    // OSRM stuff
    //--------------------------------------------------------------------
    double getCostOsrm() const;
    double getTotTravelTimeOsrm() const;
    void evaluateOsrm();
    #endif
#endif
    const Trashnode& operator[]( int i ) const { return path[i]; };
    Trashnode&  operator[](int i)  { return path[i]; };

    #ifdef DOVRPLOG
    //--------------------------------------------------------------------
    // dumps 
    //--------------------------------------------------------------------
    void dump() const;
    void dump( const std::string &title ) const;
    void dumpeval() const;
    void smalldump() const;
    void dumppath() const;
    void tau() const ;
    #endif

    #ifdef DOPLOT
    //--------------------------------------------------------------------
    // plots
    //--------------------------------------------------------------------
    void plot( std::string file, std::string title, int carnumber ) const;
    void plot( Plot<Trashnode> graph, int carnumber ) const;
    #endif

    //--------------------------------------------------------------------
    // evaluation
    //--------------------------------------------------------------------

    /**
    For the truck to be feasable the following stops must be feasable:
    last node in path
    dumpSite
    endingSite
    */
    bool feasable() const { return path.feasable() and dumpSite.feasable() and endingSite.feasable(); };
    bool hascv()const { return endingSite.cvTot() != 0; };
    bool hastwv()const { return endingSite.twvTot() != 0; };

    void evalLast();
    void evaluate();

    //--------------------------------------------------------------------
    // mutators
    //--------------------------------------------------------------------

    // these two do not work with autoeval
    // instead use BaseVehicle(depot, dump) constructor
    //void setdepot(Trashnode _depot) { endingSite = _depot; };
    //void setdumpSite(Trashnode _dump) { dumpSite = _dump; };

    void setweights( double _w1, double _w2, double _w3 ) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };

    //--------------------------------------------------------------------
    // wrappers to twpath code to
    //--------------------------------------------------------------------

    // single path manipulators

    bool push_back( Trashnode node );
    bool insert( Trashnode node, int at );
#if 0
    bool push_front( Trashnode node );
    bool remove( int at );
    bool moverange( int rangefrom, int rangeto, int destbefore );
    bool movereverse( int rangefrom, int rangeto, int destbefore );
    bool reverse( int rangefrom, int rangeto );
    bool move( int fromi, int toj );
    bool swap( const int &i, const int &j );

    // multiple path manipulators

    bool swap( BaseVehicle &v2, const int &i1, const int &i2 );

    // restore a saved path to undo an operation

    void restorePath( Twpath<Trashnode>
                      oldpath ); //tmp=v; v.dostuff; v=tmp restores as original

#endif

#if 0
    //--------------------------------------------------------------------
    // algorithm specific - intra-route manipulations
    //--------------------------------------------------------------------

    bool doTwoOpt( const int &c1, const int &c2, const int &c3, const int &c4 );
    bool doThreeOpt( const int &c1, const int &c2, const int &c3, const int &c4,
                     const int &c5, const int &c6 );
    bool doOrOpt( const int &c1, const int &c2, const int &c3 );
    bool doNodeMove( const int &i, const int &j );
    bool doNodeSwap( const int &i, const int &j );
    bool doInvertSeq( const int &i, const int &j );

    bool pathOptimize();
    bool pathTwoOpt();
    bool pathThreeOpt();
    bool pathOrOpt();
    bool pathOptMoveNodes();
    bool pathOptExchangeNodes();
    bool pathOptInvertSequence();

    bool findBestFit( const Trashnode &node, int *tpos, double *deltacost );

    //--------------------------------------------------------------------
    // algorithm specific - inter-route manipulations
    //--------------------------------------------------------------------

    bool swap2( BaseVehicle &v2, const int &i1, const int &i2, bool force );
    bool swap3( BaseVehicle &v2, BaseVehicle &v3, const int &i1, const int &i2,
                const int &i3, bool force );
    bool exchangeSeq( BaseVehicle &v2, const int &i1, const int &j1, const int &i2,
                      const int &j2, bool force );
    bool exchangeTails( BaseVehicle &v2, const int &i1, const int &i2, bool force );
    bool exchange3( BaseVehicle &v2, BaseVehicle &v3, const int &cnt, const int &i1,
                    const int &i2, const int &i3, bool force );
    bool relocate( BaseVehicle &v2, const int &i1, const int &i2, bool force );
    bool relocateBest( BaseVehicle &v2, const int &i1 );
#endif



    //----------------------------------------------------------------
    // I really hate these shortcuts & I love them but I'll think about them really hard
    //----------------------------------------------------------------

    Bucket  Path() const { return path; };
    inline int nid(int i) const { return path[i].nid(); };
    inline int id(int i) const { return path[i].id(); };
    inline double x(const int i) const { return path[i].x(); };
    inline double y(const int i) const { return path[i].y(); };
    bool hasDemand(int i) const { return path[i].hasDemand(); };
    bool hasSupply(int i) const { return path[i].hasSupply(); };
    bool hasNoGoods(int i) const { return path[i].hasNoGoods(); };
    bool earlyArrival(int i, const double D) const { return path[i].earlyArrival(D); };
    bool lateArrival(int i, const double D) const { return path[i].lateArrival(D); };
    bool onTime(int i, const double D) const { return not earlyArrival(i, D) and not lateArrival(i, D); };
    bool isDump(int i) const { return path[i].isDump(); };
    bool isPickup(int i) const { return path[i].isPickup(); };
    bool isDepot(int i) const { return path[i].isDepot(); };
    bool cargo(int i) const { return path[i].cargo(); };


};


#endif

