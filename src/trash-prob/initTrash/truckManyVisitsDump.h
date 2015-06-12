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

#include "./basicOperations.h"


class TruckManyVisitsDump : public Basicoperations {
#if 0
 private:
  typedef  TwBucket<Trashnode> Bucket;
  typedef  unsigned long int UID;
  typedef  unsigned long int POS;


  std::deque<Vehicle> unusedTrucks;
  std::deque<Vehicle> usedTrucks;
  Bucket unassigned;
  Bucket problematic;
  Bucket assigned;
#endif
  int icase;


 public:
  void process(int pcase);

  explicit TruckManyVisitsDump(const std::string &infile): Basicoperations(infile) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
  }

  TruckManyVisitsDump(const Prob_trash &P): Basicoperations(P) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    twc->fill_travel_time_onTrip();
  };


 private:
#if 0
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
  Vehicle getTruck();
#endif
/*
template <class V>
void remove_CV(V &trip);
*/

  bool insertBestPairInCleanTrip(Trip &trip);
  bool insertBestPairSubPath(std::deque<Trip> &trips);
  bool insertBigSubPathAtBegin(Trip &trip);
  bool insertBigSubPathAtEnd(Trip &trip);
  void deleteNodesOfTrip(Trip &trip, int cant);
  void remove_CV(Trip &trip);
  void remove_TWV(Trip &trip);
  void initializeTrip(Trip &trip, bool fromStart);
  void fillTrip(Trip &trip);
  void add_extra_trip(Vehicle &truck);
  void buildTruck(Vehicle &truck, std::deque<Trip> &trips);
  void initializeTruck(Vehicle &truck, std::deque<Trip> &trips);
  void fillTruck(Vehicle &truck, std::deque<Trip> &trips);
  void fillFleet();
  void insertNodesOnPath(Trip &trip);
  void fillOneTruck(Vehicle &truck);
  bool insertTrip(Trip &trip, Vehicle &truck);
  void IntraSwMoves(Vehicle &truck);
};

#endif  // SRC_TRASH_PROB_INITTRASH_TRUCKMANYVISITSDUMP_H_
