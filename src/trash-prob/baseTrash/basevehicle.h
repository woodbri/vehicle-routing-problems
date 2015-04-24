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


#include "basictypes.h"
#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
//#include "move.h"
#include "pg_types_vrp.h"


class BaseVehicle
{
protected:
  typedef  TwBucket<Trashnode> Bucket;

  int vid;
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

public:
  bool isvalid() const {return vid >= 0;};  // more complicated than this
  bool findNearestNodeTo(Bucket &unassigned, POS &pos, Trashnode &bestNode);
  bool findFastestNodeTo(Bucket &unassigned, POS &pos, Trashnode &bestNode);
  bool e_setPath(const Bucket &sol);
  void setTravelingTimesOfRoute() const;

  //--------------------------------------------------------------------
  // constructors
  //--------------------------------------------------------------------

  BaseVehicle();
  BaseVehicle(int vid, int start_id, int dump_id, int _nd_id,
              double capacity, double dumpservicetime, double starttime,
              double endtime, const Bucket &otherlocs);
  BaseVehicle(const std::string &line, const Bucket &otherlocs);


  //--------------------------------------------------------------------
  // accessors
  //--------------------------------------------------------------------

  Twpath<Trashnode> getvpath() const { return path; };
  Twpath<Trashnode> &getvpath() { return path; };
  std::deque<int> getpath() const;
  UINT size() const { return path.size(); };
  double getmaxcapacity() const { return maxcapacity; };
  int twvTot() const { return endingSite.twvTot(); };
  int cvTot() const { return endingSite.cvTot(); };
  double getCargo() const { return  path.cargo(); };
  double getDuration() const { return ( path.size() - 1 == 0 ) ? 0.0 : endingSite.getTotTime(); };
  double getcost() const { return cost; };
  double getw1() const { return w1; };
  double getw2() const { return w2; };
  double getw3() const { return w3; };
  int getVid() const { return vid; };
  //    inline double getCurrentCapacity() const {return (maxcapacity - path.cargo());}
  const Trashnode &getDepot() const { return path[0]; };
  const Trashnode &getStartingSite() const {return path[0];}
  const Trashnode &getDumpSite() const { return dumpSite; };
  const Trashnode &getEndingSite() const {return endingSite;}
  Trashnode &getDepot() { return path[0]; };
  Trashnode &getStartingSite() {return path[0];}
  Trashnode &getDumpSite()  { return dumpSite; };
  Trashnode &getEndingSite()  {return endingSite;}

  const Trashnode &operator[]( int i ) const { return path[i]; };
  Trashnode  &operator[](int i)  { return path[i]; };

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
  void evaluateOsrm();

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

  bool push_back(Trashnode node);
  bool e_insert(Trashnode node, int at);
  bool e_remove(int at);
  void e_swap(int i,int j);
  //void e_insert(Trashnode, int i);

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

