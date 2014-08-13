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

    std::vector< std::vector<double> > dMatrix;
    double extents[4];

  public:

    double distance(int nq, int n2) const;

    void loadproblem(std::string& file);
    void setNodeDistances(Trashnode& n);

    void buildDistanceMatrix();

    // get solution
    std::string solutionAsText() const;
    std::vector<int> solutionAsVector() const;

    // methods to build initial solution
    void nearestNeighbor();
    void nearestInsertion();
    void farthestInsertion();
    void assignmentSweep();

    // optimization routines
    void opt_2opt();

    void dumpDmatrix() const;
    void dumpFleet() const;
    void dumpdataNodes() const;
    void dumpDepots() const;
    void dumpDumps() const;
    void dumpPickups() const;
    void dump() const;

};

#endif
