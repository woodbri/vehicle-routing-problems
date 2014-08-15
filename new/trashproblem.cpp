
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "trashproblem.h"

double TrashProblem::distance(int n1, int n2) const {
    return datanodes[n1].distance(datanodes[n2]);
}


void TrashProblem::loadproblem(std::string& file) {
    std::ifstream in( file.c_str() );
    std::string line;

    // initialize the extents
    extents[0] = std::numeric_limits<double>::max();
    extents[1] = std::numeric_limits<double>::max();
    extents[2] = std::numeric_limits<double>::min();
    extents[3] = std::numeric_limits<double>::min();

    // read the nodes
    int cnt = 0;
    while ( std::getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node( line );
        if (!node.isvalid())
            std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;

        // compute the extents as we load the data for plotting
        if (node.getx() < extents[0]) extents[0] = node.getx();
        if (node.gety() < extents[1]) extents[1] = node.gety();
        if (node.getx() > extents[2]) extents[2] = node.getx();
        if (node.gety() > extents[3]) extents[3] = node.gety();

        datanodes.push_back(node);

        if (node.ispickup())
            pickups.push_back(node.getnid());
        else if (node.isdepot())
            depots.push_back(node.getnid());
        else if (node.isdump())
            dumps.push_back(node.getnid());
    }

    in.close();

    // add a small buffer around the extents
    extents[0] -= (extents[2] - extents[0]) * 0.02;
    extents[2] += (extents[2] - extents[0]) * 0.02;
    extents[1] -= (extents[3] - extents[1]) * 0.02;
    extents[3] += (extents[3] - extents[1]) * 0.02;

    buildDistanceMatrix();

    for (int i=0; i<datanodes.size(); i++)
        setNodeDistances(datanodes[i]);
}


void TrashProblem::setNodeDistances(Trashnode& n) {
    double dist = -1.0;
    int nid = -1;
    double dist2 = -1.0;
    int nid2 = -1;

    if (n.isdepot()) {
        n.setdepotdist(n.getnid(), 0.0, -1, -1.0);
        for (int i=0; i<dumps.size(); i++) {
            double d = dMatrix[n.getnid()][dumps[i]];
            if (nid == -1 or d < dist) {
                dist = d;
                nid = dumps[i];
            }
        }
        n.setdumpdist(nid, dist);
    }
    else if (n.isdump()) {
        n.setdumpdist(n.getnid(), 0.0);
        for (int i=0; i<depots.size(); i++) {
            double d = dMatrix[n.getnid()][depots[i]];
            if (nid == -1 or d < dist) {
                dist = d;
                nid = depots[i];
            }
        }
        n.setdepotdist(nid, dist, -1, -1.0);
    }
    else if (n.ispickup()) {
        for (int i=0; i<dumps.size(); i++) {
            double d = dMatrix[n.getnid()][dumps[i]];
            if (nid == -1 or d < dist) {
                dist = d;
                nid = dumps[i];
            }
        }
        n.setdumpdist(nid, dist);

        nid = -1;
        for (int i=0; i<depots.size(); i++) {
            double d = dMatrix[n.getnid()][depots[i]];
            if (nid == -1 or d < dist) {
                dist2 = dist;
                nid2 = nid;
                dist = d;
                nid = depots[i];
            }
        }
        n.setdepotdist(nid, dist, nid2, dist2);
    }
}


void TrashProblem::buildDistanceMatrix() {
    dMatrix.clear();
    dMatrix.resize(datanodes.size());
    for (int i=0; i<datanodes.size(); i++) {
        dMatrix[i].clear();
        dMatrix[i].resize(datanodes.size());
        for (int j=0; j<datanodes.size(); j++) {
            dMatrix[i][j] = datanodes[i].distance(datanodes[j]);
        }
    }
}

// search for node methods

// selector is a bit mask (TODO: make these an enum)
// selector: 0 - any
//           1 - must be unassigned
//           2 - in nid's cluster1
//           4 - in nid's cluster2
//           8 - 
//          16 - must be pickup nodes
//          32 - must be depot nodes
//          64 - must be dump nodes

int TrashProblem::findNearestNodeTo(int nid, int selector) {
    Trashnode &tn(datanodes[nid]);
    int nn = -1;    // init to not found
    double dist = -1;    // dist to nn

    for (int i=0; i<datanodes.size(); i++) {
        bool select = false;

        // if select any
        if (!selector)
            select = true;
        // is pickup node
        else if (selector & 16 and datanodes[i].ispickup())
            select = true;
        // is depot node
        else if (selector & 32 and datanodes[i].isdepot())
            select = true;
        // is dump node
        else if (selector & 64 and datanodes[i].isdump())
            select = true;
        // is unassigned node
        else if (selector & 1 and unassigned[i])
            select = true;
        // belongs to cluster1
        else if (selector & 2 and (
                 tn.getdepotnid() == datanodes[i].getdepotnid() or
                 tn.getdepotnid() == datanodes[i].getdepotnid2() ) )
                select = true;
        // belongs to cluster2
        else if (selector & 4 and (
                 tn.getdepotnid2() == datanodes[i].getdepotnid() or
                 tn.getdepotnid2() == datanodes[i].getdepotnid2() ) )
                select = true;
        else
            continue;

        double d = dMatrix[tn.getnid()][i];
        if (nn == -1 or d < dist) {
            dist = d;
            nn = i;
        }
    }
    std::cout << "TrashProblem::findNearestNodeTo(" << nid << ", " << selector << ") = " << nn << " at dist = " << dist << std::endl;
    return nn;
}


std::string TrashProblem::solutionAsText() {
    std::stringstream ss;;
    std::vector<int> s = solutionAsVector();
    for (int i=0; i<s.size(); i++) {
        if (i) ss << ",";
        ss << s[i];
    }
    return ss.str();
}


std::vector<int>  TrashProblem::solutionAsVector() {
    std::vector<int> s;
    for (int i=0; i<fleet.size(); i++) {
        if (fleet[i].size() == 0) continue;
        for (int j=0; j<fleet[i].size(); j++) {
            s.push_back(fleet[i][j].getnid());
        }
        s.push_back(-1);
    }
    return s;
}


void TrashProblem::nearestNeighbor() {
    // create a list of all pickup nodes and make them unassigned
    unassigned = std::vector<int>(datanodes.size(), 1);

    for (int i=0; i<depots.size(); i++) {
        Vehicle truck;

        // add this depot as the vehicles home location
        Trashnode& depot(datanodes[depots[i]]);
        truck.setdepot(depot);
        // add the closest dump for now, this might change later
        truck.setdumpsite(datanodes[depot.getdumpnid()]);

        // remember the last node we inserted
        Trashnode& last_node = datanodes[depots[i]];

        truck.evaluate();

        while (truck.getcurcapacity() <= truck.getmaxcapacity()) {

            // add the nearest unassigned node to path after the last node
            int nnid = -1;
            double ndist;
            for (int j=0; j<pickups.size(); j++) {
                // skip if assigned to another truck
                if (! unassigned[pickups[j]]) continue;
                // skip if adding this node exceeds capacity
                if (truck.getcurcapacity() + datanodes[pickups[j]].getdemand()
                    > truck.getmaxcapacity()) continue;

                double d = dMatrix[pickups[j]][last_node.getnid()];
                if (nnid == -1 or d < ndist) {
                    nnid = pickups[j];
                    ndist = d;
                }
            }
            // if we did not find a node we can add break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            truck.push_back(datanodes[nnid]);
            truck.evaluate();
        }
        std::cout << "nearestNeighbor: depot: " << i << std::endl;
        truck.dump();
        fleet.push_back(truck);
    }
    // report unassigned nodes
    std::cout << "-------- Unassigned after TrashProblem::nearestNeighbor\n";
    for (int i=0; i<pickups.size(); i++) {
        if (unassigned[pickups[i]])
            std::cout << "    " << pickups[i] << std::endl;
    }
}





void TrashProblem::nearestInsertion() {

}


void TrashProblem::farthestInsertion() {

}


void TrashProblem::assignmentSweep() {

}


void TrashProblem::opt_2opt() {

}

void TrashProblem::dumpDmatrix() const {
    std::cout << "--------- dMatrix ------------" << std::endl;
    for (int i=0; i<dMatrix.size(); i++) {
        for (int j=0; j<dMatrix[i].size(); j++) {
            std::cout << i << "\t" << j << "\t" << dMatrix[i][j] << std::endl;
        }
    }
}


void TrashProblem::dumpFleet() {
    std::cout << "--------- Fleet ------------" << std::endl;
    for (int i=0; i<fleet.size(); i++)
        fleet[i].dump();
}


void TrashProblem::dumpdataNodes() const {
    std::cout << "--------- Nodes ------------" << std::endl;
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}


void TrashProblem::dumpDepots() const {
    std::cout << "--------- Depots ------------" << std::endl;
    for (int i=0; i<depots.size(); i++)
        datanodes[depots[i]].dump();
}


void TrashProblem::dumpDumps() const {
    std::cout << "--------- Dumps ------------" << std::endl;
    for (int i=0; i<dumps.size(); i++)
        datanodes[dumps[i]].dump();
}


void TrashProblem::dumpPickups() const {
    std::cout << "--------- Pickups ------------" << std::endl;
    for (int i=0; i<pickups.size(); i++)
        datanodes[pickups[i]].dump();
}


void TrashProblem::dump() {
    dumpDepots();
    dumpDumps();
    dumpPickups();
    dumpFleet();
    std::cout << "Solution: " << solutionAsText() << std::endl;
}

