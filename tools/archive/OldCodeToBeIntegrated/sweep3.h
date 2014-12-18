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



class Sweep3 : public Solution {
  private:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID;

    double ratio; // ratio for select boarder nodes in assignmentSweep3

    std::deque<Vehicle> unusedTrucks;
    std::deque<Vehicle> usedTrucks;
    Bucket unassigned;
    Bucket problematic;
    Bucket assigned;


int tmp;
  public:

    Sweep3(const std::string &infile): Solution(infile) { 
       unusedTrucks=trucks;
       unassigned=pickups;
       fleet.clear();
tmp=0;
       ratio = 0.85; 
       assignmentSweep3();
    };


 private:
    void stepOne(Vehicle &truck);
    Vehicle getTruck();
    void assignmentSweep3();

    // get solution
//    std::string solutionAsText() const;
//    std::vector<int> solutionAsVector() const;

    double getduration() const;
    double getcost() const;
    int getTWV() const;
    int getCV() const;
    double getRatio() const { return ratio; };

    int getVehicleCount() const {return fleet.size(); };

    Vehicle getVehicle(int i) const { return fleet[i]; };

   
    void dumpFleet() const;
    void dump() const;
    void dumpSummary() const;


    // mutators
    void clearFleet() { fleet.clear(); };
    void setRatio(double r) { ratio = r; };


    
    // methods to build initial solution
    bool buildFleetFromSolution(std::vector<int> solution);
    int findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int& pos);
    bool findVehicleBestFit(const Trashnode& node, int& vid, int& pos) ;

    // intra-route optimization routines
    void opt_2opt();
    void opt_3opt();
    void opt_or_opt();
    void optimize();

};

#endif
