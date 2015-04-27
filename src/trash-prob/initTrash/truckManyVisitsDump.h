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
#ifndef TRUCKMANYVISITSDUMP_H
#define TRUCKMANYVISITSDUMP_H

#include <string>
#include <iostream>
#include <vector>

#include "trashnode.h"
#include "prob_trash.h"
#include "twbucket.h"
#include "solution.h"
#include "vehicle.h"


class TruckManyVisitsDump : public Solution {
private:
  typedef  TwBucket<Trashnode> Bucket;
  typedef  unsigned long int UID;
  typedef  unsigned long int POS;


  std::deque<Vehicle> unusedTrucks;
  std::deque<Vehicle> usedTrucks;
  Bucket unassigned;
  Bucket problematic;
  Bucket assigned;


  int tmp;
public:

  TruckManyVisitsDump(const std::string &infile): Solution(infile) {
    unusedTrucks = trucks;
    unassigned = pickups;
    tmp = 0;
    process();
  };
#if 0
  TruckManyVisitsDump(const Solution &sol): Solution(sol) {
    unusedTrucks = trucks;
    unassigned = pickups;
    tmp = 0;
    process();
  };
#endif

private:
  void fillOneTruck(Vehicle &truck, Bucket &unassigned, Bucket &assigned);
  void insertTrip(Vehicle &trip, Vehicle &truck);
  void insertGoing( Bucket &bigTruck, Vehicle &truck, UID goingPos );
  void insertComming( Bucket &bigTruck, Vehicle &truck, UID goingPos );
  double e_evalIntraSw(Vehicle &truck, POS i, POS j);
  double e_evalIns(Vehicle &truck, POS i, POS j);
  void IntraSwMoves(Vehicle &truck);
  void InsMoves(Vehicle &truck);
  Vehicle getTruck();
  void process();

};

#endif
