#ifndef FEASABLESOLLOOP_H
#define FEASABLESOLLOOP_H

#include <string>
#include <iostream>
#include <vector>

#include "twc.h"
#include "trashnode.h"
#include "prob_trash.h"
#include "solution.h"
#include "vehicle.h"



class FeasableSolLoop : public Solution {
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

    FeasableSolLoop(const std::string &infile): Solution(infile) { 
       unusedTrucks=trucks;
       unassigned=pickups;
       fleet.clear();
tmp=0;
       process();
    };


    FeasableSolLoop(const Prob_trash& P): Solution(P) { 
       unusedTrucks=trucks;
       unassigned=pickups;
       fleet.clear();
tmp=0;
       process();
    };


 private:
    void stepOneLoop(Vehicle &truck);
    Vehicle getTruck();
    void process();

};

#endif
