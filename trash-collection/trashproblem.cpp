
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "vec2d.h"
#include "trashproblem.h"

double TrashProblem::distance(int n1, int n2) const {
    return datanodes[n1].distance(datanodes[n2]);
}


void TrashProblem::loadproblem(std::string& file) {
    std::ifstream in( file.c_str() );
    std::string line;

    // read the nodes
    int cnt = 0;
    while ( std::getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node( line );
        if (!node.isvalid())
            std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;

        datanodes.push_back(node);

        if (node.ispickup())
            pickups.push_back(node.getnid());
        else if (node.isdepot())
            depots.push_back(node.getnid());
        else if (node.isdump())
            dumps.push_back(node.getnid());
    }

    in.close();

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
// selector: 0 - anyi                       ANY
//           1 - must be unassigned         UNASSIGNED
//           2 - in nid's cluster1          CLUSTER1
//           4 - in nid's cluster2          CLUSTER2
//           8 - with demand <= demandLimit LIMITDEMAND
//          16 - must be pickup nodes       PICKUP
//          32 - must be depot nodes        DEPOT
//          64 - must be dump nodes         DUMP

bool TrashProblem::filterNode(const Trashnode &tn, int i, int selector, int demandLimit) {
        bool select = false;

        // filter out nodes where the demand > demandLimit
        if (selector & LIMITDEMAND and datanodes[i].getdemand() > demandLimit)
            return true;

        // if select any
        if (!selector)
            select = true;
        // is pickup node
        else if (selector & PICKUP and datanodes[i].ispickup())
            select = true;
        // is depot node
        else if (selector & DEPOT and datanodes[i].isdepot())
            select = true;
        // is dump node
        else if (selector & DUMP and datanodes[i].isdump())
            select = true;
        // belongs to cluster1
        else if (selector & CLUSTER1 and (
                 tn.getdepotnid() == datanodes[i].getdepotnid() or
                 tn.getdepotnid() == datanodes[i].getdepotnid2() ) )
                select = true;
        // belongs to cluster2
        else if (selector & CLUSTER2 and (
                 tn.getdepotnid2() == datanodes[i].getdepotnid() or
                 tn.getdepotnid2() == datanodes[i].getdepotnid2() ) )
                select = true;

        // is unassigned node
        if (selector & UNASSIGNED and ! unassigned[i])
            return true;

        if (!select)
            return true;

        return ! select;
}


int TrashProblem::findNearestNodeTo(int nid, int selector, int demandLimit) {
    Trashnode &tn(datanodes[nid]);
    int nn = -1;    // init to not found
    double dist = -1;    // dist to nn

    for (int i=0; i<datanodes.size(); i++) {

        if (filterNode(tn, i, selector, demandLimit)) continue;

        double d = dMatrix[tn.getnid()][i];
        if (nn == -1 or d < dist) {
            dist = d;
            nn = i;
        }
    }
    std::cout << "TrashProblem::findNearestNodeTo(" << nid << ", " << selector << ") = " << nn << " at dist = " << dist << std::endl;
    return nn;
}


int TrashProblem::findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int *pos) {
    Trashnode depot(v.getdepot());
    Trashnode dump(v.getdumpsite());
    int nn = -1;        // init to not found
    int loc = 0;        // position in path to insert
    double dist = -1;   // dist to nn
    double qx, qy;

    std::cout << "TrashProblem::findNearestNodeTo(V" << depot.getnid() << ", " << selector << ")\n";

    for (int i=0; i<datanodes.size(); i++) {

        if (filterNode(depot, i, selector, demandLimit)) {
            std::cout << "FILTERED: ";
            datanodes[i].dump();
            continue;
        }

        double d;
        Trashnode &last = depot;

        for (int j=0; j<v.size(); j++) {
            d = distanceFromLineSegmentToPoint(
                last.getx(), last.gety(), v[j].getx(), v[j].gety(),
                datanodes[i].getx(), datanodes[i].gety(), &qx, &qy);
            if (nn == -1 or d < dist) {
                dist = d;
                loc = j;
                nn = i;
            }
            last = v[j];
        }
        d = distanceFromLineSegmentToPoint(
            last.getx(), last.gety(), dump.getx(), dump.gety(),
            datanodes[i].getx(), datanodes[i].gety(), &qx, &qy);
        if (nn == -1 or d < dist) {
            dist = d;
            loc = v.size();
            nn = i;
        }
    }

    std::cout << "TrashProblem::findNearestNodeTo(V" << depot.getnid() << ", " << selector << ") = " << nn << " at dist = " << dist << " at pos = " << loc << std::endl;

    *pos = loc;
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

    clearFleet();

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

            int nnid = findNearestNodeTo(depot.getnid(),
                            UNASSIGNED|PICKUP|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcurcapacity());

            // if we did not find a node we can break
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
    // create a list of all pickup nodes and make them unassigned
    unassigned = std::vector<int>(datanodes.size(), 1);

    clearFleet();

    for (int i=0; i<depots.size(); i++) {
        Vehicle truck;

        // add this depot as the vehicles home location
        Trashnode& depot(datanodes[depots[i]]);
        truck.setdepot(depot);
        // add the closest dump for now, this might change later
        truck.setdumpsite(datanodes[depot.getdumpnid()]);
        truck.evaluate();

        int pos;
        int nid = findNearestNodeTo(truck, UNASSIGNED|PICKUP|CLUSTER1, 0, &pos);
        if (nid == -1) {
            std::cout << "TrashProblem::assignmentSweep failed to find an initial node for depot: " << depots[i] << std::endl;
            continue;
        }
        truck.push_back(datanodes[nid]);
        unassigned[nid] = 0;

        while (truck.getcurcapacity() <= truck.getmaxcapacity()) {

            int nnid = findNearestNodeTo(truck,
                            UNASSIGNED|PICKUP|CLUSTER1|CLUSTER2|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcurcapacity(),
                            &pos);

            // if we did not find a node we can break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            truck.push_back(datanodes[nnid]);
            if (pos == 0)
                truck.push_front(datanodes[nnid]);
            else if (pos == truck.size())
                truck.push_back(datanodes[nnid]);
            else 
                truck.insert(truck.begin()+pos, datanodes[nnid]);

            truck.evaluate();
        }
        std::cout << "assignmentSweep: depot: " << i << std::endl;
        truck.dump();
        fleet.push_back(truck);
    }
    // report unassigned nodes
    std::cout << "-------- Unassigned after TrashProblem::assignmentSweep\n";
    for (int i=0; i<pickups.size(); i++) {
        if (unassigned[pickups[i]])
            std::cout << "    " << pickups[i] << std::endl;
    }
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

