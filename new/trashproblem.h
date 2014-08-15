#ifndef TRASHPROBLEM_H
#define TRASHPROBLEM_H

#include <string>
#include <iostream>
#include <vector>

#include "trashnode.h"
#include "path.h"
#include "vehicle.h"

enum Selector {
    ANY         =0,
    UNASSIGNED  =1,
    CLUSTER1    =2,
    CLUSTER2    =4,
    LIMITDEMAND =8,
    PICKUP      =16,
    DEPOT       =32,
    DUMP        =64
};


class TrashProblem {
  private:
    std::vector<Vehicle> fleet;
    std::vector<Trashnode> datanodes;
    std::vector<int> depots;
    std::vector<int> dumps;
    std::vector<int> pickups;

    std::vector<int> unassigned;

    std::vector< std::vector<double> > dMatrix;
    double extents[4];

  public:

    double distance(int nq, int n2) const;

    void loadproblem(std::string& file);
    void setNodeDistances(Trashnode& n);

    void buildDistanceMatrix();

    // selector is a bit mask (TODO: make these an enum)
    // selector: 0 - any
    //           1 - must be unassigned
    //           2 - in nid's cluster1
    //           4 - in nid's cluster2
    //           8 - with demand < demandLimit
    //          16 - must be pickup nodes
    //          32 - must be depot nodes
    //          64 - must be dump nodes

    int findNearestNodeTo(int nid, int selector, int demandLimit);

    // get solution
    std::string solutionAsText();
    std::vector<int> solutionAsVector();

    // methods to build initial solution
    void nearestNeighbor();
    void nearestInsertion();
    void farthestInsertion();
    void assignmentSweep();

    // optimization routines
    void opt_2opt();

    void dumpDmatrix() const;
    void dumpFleet();
    void dumpdataNodes() const;
    void dumpDepots() const;
    void dumpDumps() const;
    void dumpPickups() const;
    void dump();

};

#endif
