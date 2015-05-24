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
#ifndef SRC_TRASH_PROB_INITTRASH_TRUCKMANYVISITSDUMP_H_
#define SRC_TRASH_PROB_INITTRASH_TRUCKMANYVISITSDUMP_H_

#include <string>
#include <deque>

#include "./solution.h"
#include "signalhandler.h"


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


 public:
  explicit TruckManyVisitsDump(const std::string &infile): Solution(infile) {
    unusedTrucks = trucks;
    unassigned = pickups;
    process();
  }

  TruckManyVisitsDump( const Prob_trash &P ): Solution( P ) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    process();
  };


 private:
  void fillOneTruck(Vehicle &truck);
  bool insertTrip(Vehicle &trip, Vehicle &truck);
  void IntraSwMoves(Vehicle &truck);
  Vehicle getTruck();
  void process();
};

#endif  // SRC_TRASH_PROB_INITTRASH_TRUCKMANYVISITSDUMP_H_
