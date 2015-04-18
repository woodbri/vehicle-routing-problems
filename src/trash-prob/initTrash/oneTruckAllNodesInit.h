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
#ifndef  ONETRUCKALLNODESINIT_H
#define  ONETRUCKALLNODESINIT_H

#include <string>
#include <iostream>
#include <vector>

#include "trashnode.h"
#include "prob_trash.h"
#include "solution.h"
#include "vehicle.h"



class OneTruckAllNodesInit : public Solution
{
private:
  typedef  TwBucket<Trashnode> Bucket;
  typedef  unsigned long int UID;



public:

  OneTruckAllNodesInit( const std::string &infile ): Solution( infile ) {
    process();
  };




  // get solution
  std::string solutionAsText() const;
  std::vector<int> solutionAsVector() const;

  Vehicle getVehicle( int i ) const { return fleet[i]; };


  void dumpFleet() const;
  void dump() const;
  void dumpSummary() const;


  // mutators
  void clearFleet() { fleet.clear(); };



  // methods to build initial solution
  void stepOne(Vehicle &truck, Bucket &unassigned, Bucket &assigned);
  void process();

};

#endif
