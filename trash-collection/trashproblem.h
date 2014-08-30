#ifndef TRASHPROBLEM_H
#define TRASHPROBLEM_H

#include <string>
#include <iostream>
#include <vector>

#include "trashnode.h"
#include "vehicle.h"

enum Selector {
    ANY         =0,     // any
    UNASSIGNED  =1,     // must be unassigned
    CLUSTER1    =2,     // in nid's cluster1
    CLUSTER2    =4,     // in nid's cluster2
    LIMITDEMAND =8,     // with demand < demandLimit
    PICKUP      =16,    // must be pickup nodes
    DEPOT       =32,    // must be depot nodes
    DUMP        =64     // must be dump nodes
};


class TrashProblem {
  private:
    std::vector<Vehicle> fleet;
    Twpath<Trashnode> datanodes;
    std::deque<int> depots;
    std::deque<int> dumps;
    std::deque<int> pickups;

    std::vector<int> unassigned;

    std::vector< std::vector<double> > dMatrix;

  public:
    // accessors
    double distance(int nq, int n2) const;

    bool filterNode(const Trashnode &tn, int i, int selector, int demandLimit);

    //// these should be const
    int findNearestNodeTo(int nid, int selector, int demandLimit);
    int findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int *pos);

    // get solution
    std::string solutionAsText();           //// const
    std::vector<int> solutionAsVector();    //// const

    double getduration();
    double getcost();
    int getTWV();
    int getCV();

    void dumpDmatrix() const;
    void dumpFleet();                       //// const
    void dumpdataNodes() const;
    void dumpDepots() const;
    void dumpDumps() const;
    void dumpPickups() const;
    void dump();                            /// const

    void plot( std::string file, std::string title, std::string font );
    void plot( std::string file, std::string title, std::string font, std::deque<int> highlight );

    // mutators
    void loadproblem(std::string& file);
    void setNodeDistances(Trashnode& n);

    void buildDistanceMatrix();

    // methods to build initial solution
    void clearFleet() { fleet.clear(); };
    void dumbConstruction();
    void nearestNeighbor();
    void nearestInsertion();
    void farthestInsertion();
    void assignmentSweep();
    void assignmentSweep2();

    // optimization routines
    void opt_2opt();
    void opt_3opt();

};

#endif
