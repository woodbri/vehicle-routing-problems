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
#ifndef STREET_H
#define STREET_H

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


  protected:
  inline double MAX() { (std::numeric_limits<double>::max()); };


  public:

    //--------------------------------------------------------------------
    // structors
    //--------------------------------------------------------------------

    Street() {
       sid=-1;
    };

    Street( const Trashnode &node) {
        sid=node.streetId();
        e_push_back( node );
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
    void dumpid() const;
    void dump(const std::string &title) const;
    void dumpeval() const ;
    void smalldump() const;
    void dumppath() const;
    void tau() const ;

    void plot(std::string file,std::string title);
    void plot(Plot<Trashnode> graph);


    bool e_push_back(const Trashnode &node);
    bool e_push_front(const Trashnode &node);
    bool insert(const Trashnode &node);
    int getBestPos(const Trashnode&);
    bool e_insert(const Trashnode &node);
    int e_insert(const Bucket& containers);
    int e_insert(Bucket& unassigned, Bucket &assigned);

    void evalLast();


    //--------------------------------------------------------------------
    // wrappers to twpath code to 
    //--------------------------------------------------------------------
    
/*
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

