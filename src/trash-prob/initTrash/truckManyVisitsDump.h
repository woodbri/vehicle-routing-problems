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
  int icase;


 public:
  void process(int pcase);

  explicit TruckManyVisitsDump(const std::string &infile): Solution(infile) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    twc->fill_travel_time_onTrip();
  }

  TruckManyVisitsDump( const Prob_trash &P ): Solution( P ) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    twc->fill_travel_time_onTrip();
  };


 private:
bool safeInsertNode(Vehicle &trip, const Trashnode &node, UINT pos);
bool safeDeleteNode(Vehicle &trip, UINT pos);
bool safeInsertSubpath(Vehicle &trip, Bucket &subPath, UINT pos);
bool safePushFrontSubpath(Vehicle &trip, Bucket &subPath);
bool safePushBackSubpath(Vehicle &trip, Bucket &subPath);
bool safePushBackNode(Vehicle &trip, Trashnode &node);
bool safePushFrontNode(Vehicle &trip, Trashnode &node);
bool safePopBackNode(Vehicle &trip); 
bool safePopFrontNode(Vehicle &trip); 

  void invariant();
  bool insertBestPairInCleanTrip(Vehicle &trip);
  bool insertBigSubPathAtBegin(Vehicle &trip);
  bool insertBigSubPathAtEnd(Vehicle &trip);
  void deleteTrip(Vehicle &trip);
  void remove_CV(Vehicle &trip);
  void remove_TWV(Vehicle &trip);
  void initializeTrip(Vehicle &trip, bool fromStart);
  void fillTrip(Vehicle &trip);
  void buildTruck(Vehicle &truck, std::deque<Vehicle> &trips);
  void initializeTruck(Vehicle &truck, std::deque<Vehicle> &trips);
  void fillTruck(Vehicle &truck, std::deque<Vehicle> &trips);
  void fillFleet();
  void insertNodesOnPath(Vehicle &trip);
  void fillOneTruck(Vehicle &truck);
  bool insertTrip(Vehicle &trip, Vehicle &truck);
  void IntraSwMoves(Vehicle &truck);
  Vehicle getTruck();
};

#endif  // SRC_TRASH_PROB_INITTRASH_TRUCKMANYVISITSDUMP_H_
