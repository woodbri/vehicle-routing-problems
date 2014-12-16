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
#include "solution.h"
#include "vehicle.h"
#include "oneTruckAllNodesInit.h"


class TruckManyVisitsDump : public Solution {
  private:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID;


    std::deque<Vehicle> unusedTrucks;
    std::deque<Vehicle> usedTrucks;
    Bucket unassigned;
    Bucket problematic;
    Bucket assigned;


    int tmp;
  public:

    TruckManyVisitsDump( const Solution &sol ): Solution( sol ) {
        unusedTrucks = trucks;
        unassigned = pickups;
        tmp = 0;
        process();
    };


  private:
    void insertGoing( Bucket &bigTruck, Vehicle &truck, UID goingPos );
    void insertComming( Bucket &bigTruck, Vehicle &truck, UID goingPos );
    Vehicle getTruck();
    void process();

};

#endif
