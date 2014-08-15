#ifndef TRASHPROBLEM_H
#define TRASHPROBLEM_H

#include <string>
#include <iostream>
#include <vector>

#include "trashnode.h"
#include "path.h"
#include "vehicle.h"


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

    // search for node methods
    // filter: 0 - unfiltered
    //         1 - unassigned only
    //         2 - only those nodes in nid's cluster
    //         3 - 1 and 2
    int findNearestNodeTo(int nid, int filter);

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
