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
    void loadproblem(std::string file);

    void buildDistanceMatrix();

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

    void dumpDmatrix();
    void dumpFleet();
    void dumpdataNodes();
    void dumpDepots();
    void dumpDumps();
    void dumpPickups();
    void dump() const;

};

#endif
