#ifndef TRASHPROBLEM_H
#define TRASHPROBLEM_H

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

/*
    double getduration() const;
    double getcost() const;
    int getTWV() const;
    int getCV() const;

    int getVehicleCount() const {return fleet.size(); };

    Vehicle getVehicle(int i) const { return fleet[i]; };

    bool findVehicleBestFit(const Trashnode& node, int& vid, int& pos) ;
   
    void dumpFleet() const;
    void dump() const;
    void dumpSummary() const;


    // mutators
    void clearFleet() { fleet.clear(); };
    void setRatio(double r) { ratio = r; };


    
    // methods to build initial solution
//    bool buildFleetFromSolution(std::vector<int> solution);
//    int findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int& pos);

    // intra-route optimization routines
    void opt_2opt();
    void opt_3opt();
    void opt_or_opt();
    void optimize();
*/
};

#endif
