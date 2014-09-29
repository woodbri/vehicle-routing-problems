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


  public:

    Sweep3(const std::string &infile): Solution(infile) { 
       ratio = 0.85; 
       assignmentSweep3();
    };




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
    void stepOne(Vehicle &truck, Bucket &unassigned, Bucket &assigned);
    void assignmentSweep3();
    int findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int& pos);
    bool findVehicleBestFit(int nid, int& vid, int& pos) ;

    // intra-route optimization routines
    void opt_2opt();
    void opt_3opt();
    void opt_or_opt();
    void optimize();

};

#endif
