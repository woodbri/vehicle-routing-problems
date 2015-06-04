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
#ifndef SRC_TRASH_PROB_INITTRASH_BASICOPERATIONS_H_
#define SRC_TRASH_PROB_INITTRASH_BASICOPERATIONS_H_

#include <string>
#include <deque>

#include "./solution.h"


class Basicoperations : public Solution {
 protected:
  typedef  TwBucket<Trashnode> Bucket;
  typedef  unsigned long int UID;
  typedef  unsigned long int POS;


  std::deque<Vehicle> unusedTrucks;
  std::deque<Vehicle> usedTrucks;
  Bucket unassigned;
  Bucket problematic;
  Bucket assigned;


 public:
  explicit Basicoperations(const std::string &infile): Solution(infile) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
  }

  Basicoperations( const Prob_trash &P ): Solution( P ) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    twc->fill_travel_time_onTrip();
  };


 protected:
  bool safeInsertNode(Trip &trip, const Trashnode &node, UINT pos);
  bool safeDeleteNode(Trip &trip, UINT pos);
  bool safeInsertSubpath(Trip &trip, Bucket &subPath, UINT pos);
  bool safePushFrontSubpath(Trip &trip, Bucket &subPath);
  bool safePushBackSubpath(Trip &trip, Bucket &subPath);
  bool safePushBackNode(Trip &trip, Trashnode &node);
  bool safePushFrontNode(Trip &trip, Trashnode &node);
  bool safePopBackNode(Trip &trip); 
  bool safePopFrontNode(Trip &trip); 
  Vehicle getTruck();
  void invariant();
};

#endif  // SRC_TRASH_PROB_INITTRASH_BASICOPERATIONS_H_
