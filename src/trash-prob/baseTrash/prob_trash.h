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
#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

#include "tripVehicle.h"
#include "twpath.h"
#include "twc.h"

class Prob_trash
{
protected:
  typedef  TwBucket<Trashnode> Bucket;
  typedef  unsigned long int UID ;
  typedef  unsigned long int POS ;
  typedef  unsigned long int UINT;
  inline double _MAX() {return ( std::numeric_limits<double>::max() ); };
  inline double _MIN() {return ( - std::numeric_limits<double>::max() ); };


  //    Trashnode depot;

  Twpath<Trashnode> datanodes; //dissallowing set operations
  Bucket otherlocs;
  Bucket dumps;
  Bucket depots;
  Bucket pickups;
  Bucket endings;
  Bucket invalid;
  std::deque<Vehicle> trucks;
  std::deque<Vehicle> invalidTrucks;
  Trashnode C;

  std::string datafile;


public:

  void clear() {
    otherlocs.clear();
    dumps.clear();
    depots.clear();
    pickups.clear();
    endings.clear();
    invalid.clear();
    trucks.clear();
    invalidTrucks.clear();
  }

  //    Trashnode getdepot() const { return depot;};
  Prob_trash() {};
  Prob_trash( const char *infile );
  Prob_trash( const std::string &infile );
  void loadProblem( const std::string &infile );

  unsigned int getNodeCount() const {return datanodes.size();};

  bool checkIntegrity() const;


  double distance( int n1, int n2 ) const;
  double nodeDemand( int i ) const;
  double nodeServiceTime( int i ) const;
  bool earlyArrival( int nid, double D ) const;
  bool lateArrival( int nid, double D ) const;

  void twcijDump() const;


#ifdef DOVRPLOG
  void nodesdump();
  void nodesdumpeval();
  void dump();
  void dumpdataNodes() const;
  void dumpDepots() const;
  void dumpDumps() const;
  void dumpPickups() const;
#endif

#ifdef DOPLOT
  void plot( Plot<Trashnode> &graph );
#endif



private:
  void load_depots( std::string infile );
  void load_dumps( std::string infile );
  void load_pickups( std::string infile );
  void load_endings( std::string infile );
  void load_otherlocs( std::string infile );
  void load_trucks( std::string infile );



};

#endif
