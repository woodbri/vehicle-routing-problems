#ifndef FEASABLESOL_H
#define FEASABLESOL_H

#include <string>
#include <iostream>
#include <vector>

#include "twc.h"
#include "trashnode.h"
#include "prob_trash.h"
#include "solution.h"
#include "vehicle.h"



class FeasableSol : public Solution {
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

    FeasableSol(const std::string &infile): Solution(infile) { 
       unusedTrucks=trucks;
       unassigned=pickups;
       fleet.clear();
tmp=0;
       process();
    };


 private:
    void stepOne(Vehicle &truck);
    Vehicle getTruck();
    void process();

};

#endif
