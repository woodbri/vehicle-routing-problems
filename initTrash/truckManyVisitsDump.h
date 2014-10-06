#ifndef TRUCKMANYVISITSDUMP_H
#define TRUCKMANYVISITSDUMP_H

#include <string>
#include <iostream>
#include <vector>

#include "twc.h"
#include "trashnode.h"
#include "prob_trash.h"
#include "solution.h"
#include "vehicle.h"
#include "oneTruckAllNodesInit.h"


class TruckManyVisitsDump : public OneTruckAllNodesInit {
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

    TruckManyVisitsDump(const std::string &infile): OneTruckAllNodesInit(infile) { 
       unusedTrucks=trucks;
       unassigned=pickups;
tmp=0;
       process();
    };


 private:
    void insertGoing(Bucket &bigTruck, Vehicle &truck, UID goingPos);
    void insertComming(Bucket &bigTruck, Vehicle &truck, UID goingPos);
    Vehicle getTruck();
    void process();

};

#endif
