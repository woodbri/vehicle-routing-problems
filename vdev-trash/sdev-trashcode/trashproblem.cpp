
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "plot.h"
#include "trashproblem.h"

// TODO: sweepAssignment and initial construction routine
//       need to deal with TW violations and DONT YET

// compute the distance between two nodes
// this currently computes Euclidean distances
// but this could be changes to call OSRM
// and/or save and fetch the distance from a matrix

double TrashProblem::distance(int n1, int n2) const {
    return datanodes[n1].distance(datanodes[n2]);
}


// routine to cycle through all the nodes and compute there
// respective Eculidean distances and put them into a matrix

void TrashProblem::buildDistanceMatrix() {
/*    dMatrix.clear();
    dMatrix.resize(datanodes.size());
    for (int i=0; i<datanodes.size(); i++) {
        dMatrix[i].clear();
        dMatrix[i].resize(datanodes.size());
        for (int j=0; j<datanodes.size(); j++) {
            dMatrix[i][j] = datanodes[i].distance(datanodes[j]);
        }
    }
*/
}


// this loads the probelm definition from a text file
// this could easily be replaced to load the problem
// from say a database query or some other format.

void TrashProblem::loadproblem(std::string& file) {
/*    std::ifstream in( file.c_str() );
    std::string line;

    // clearout any old data
    datanodes.clear();
    depots.clear();
    dumps.clear();
    pickups.clear();

    // read the nodes
    int cnt = 0;
    int nid = 0;
    while ( std::getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node( line );
        node.setnid(nid);   // renumber nodes sequentially for internal use
        if (!node.isvalid())
            std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;

        datanodes.push_back(node);

        if (node.ispickup())
            pickups.push_back(node.getnid());
        else if (node.isdepot())
            depots.push_back(node.getnid());
        else if (node.isdump())
            dumps.push_back(node.getnid());

        nid++;
    }

    in.close();

    twc.setNodes(datanodes);
twc.dump();

    buildDistanceMatrix();

    for (int i=0; i<datanodes.size(); i++)
        setNodeDistances(datanodes[i]);
*/
}


// for the input node, we find and set the nid and distance for
// the two closest depots and the nearest dump site.

void TrashProblem::setNodeDistances(Trashnode& n) {
/*    double dist = std::numeric_limits<double>::max();
    int nid = -1;
    double dist2 = std::numeric_limits<double>::max();
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
        dist = std::numeric_limits<double>::max();
        dist2 = std::numeric_limits<double>::max();
        for (int i=0; i<depots.size(); i++) {
            double d = dMatrix[n.getnid()][depots[i]];
            if (d < dist) {
                if (i and dist < dist2) {
                    dist2 = dist;
                    nid2 = nid;
                }
                dist = d;
                nid = depots[i];
            }
            else if (i and  nid2 != nid and d < dist2) {
                dist2 = d;
                nid2 = depots[i];
            }
        }
        n.setdepotdist(nid, dist, nid2, dist2);
    }
*/
}


// utility frunction to convert a selector bit array to
// human readable text for debug output

std::string selectorAsTxt(int s) {
/*    std::string str = "";
    if (s & ANY)         str += (str.length()?"|":"") + std::string("ANY");
    if (s & UNASSIGNED)  str += (str.length()?"|":"") + std::string("UNASSIGNED");
    if (s & CLUSTER1)    str += (str.length()?"|":"") + std::string("CLUSTER1");
    if (s & CLUSTER2)    str += (str.length()?"|":"") + std::string("CLUSTER2");
    if (s & LIMITDEMAND) str += (str.length()?"|":"") + std::string("LIMITDEMAND");
    if (s & PICKUP)      str += (str.length()?"|":"") + std::string("PICKUP");
    if (s & DEPOT)       str += (str.length()?"|":"") + std::string("DEPOT");
    if (s & DUMP)        str += (str.length()?"|":"") + std::string("DUMP");
    if (s & RATIO)       str += (str.length()?"|":"") + std::string("RATIO");
    return str;
*/
}


// search for node methods

// selector is a bit mask
// selector: 0 - any                        ANY
//           1 - must be unassigned         UNASSIGNED
//           2 - in nid's cluster1          CLUSTER1
//           4 - in nid's cluster2          CLUSTER2
//           8 - with demand <= demandLimit LIMITDEMAND
//          16 - must be pickup nodes       PICKUP
//          32 - must be depot nodes        DEPOT
//          64 - must be dump nodes         DUMP
//         128 - must have RATIO<ratio      RATIO
//
// return true to exclude the node
//
// tn - a depot node
// i - a datanodes index being filtered or not
// selector - see above
// demandLimit - capacity left available on the vehicle
//

bool TrashProblem::filterNode(const Trashnode &tn, int i, int selector, int demandLimit) {
/*        Trashnode& tn2 = datanodes[i];
        bool select = true;

        // filter out nodes where the demand > demandLimit
        if (selector & LIMITDEMAND and tn2.getdemand() > demandLimit)
            return true;

        // if select any
        if (!selector)
            return false;

        // is pickup node
        if (selector & PICKUP and tn2.ispickup())
            select = false;
        // is depot node
        else if (selector & DEPOT and tn2.isdepot())
            select = false;
        // is dump node
        else if (selector & DUMP and tn2.isdump())
            select = false;

        if (select and ((PICKUP|DEPOT|DUMP) & selector))
            return true;

        select = true;
        // belongs to cluster1
        if (selector & CLUSTER1 and
                tn.getdepotnid() == tn2.getdepotnid() ) {
            // select the node, it is in CLUSTER1
            select = false;
            double r = tn2.getdepotdist()<tn2.getdepotdist2() ?
                tn2.getdepotdist()/tn2.getdepotdist2() :
                tn2.getdepotdist2()/tn2.getdepotdist();
            // unselect it if r > ratio
            if (selector & RATIO and r > ratio)
                select = true;
        }
        // belongs to cluster2
        else if (selector & CLUSTER2 and 
                tn.getdepotnid() == tn2.getdepotnid2() ) {
            // select the node, it is in CLUSTER2
            select = false;
            double r = tn2.getdepotdist()<tn2.getdepotdist2() ?
                tn2.getdepotdist()/tn2.getdepotdist2() :
                tn2.getdepotdist2()/tn2.getdepotdist();
            // unselect it if r > ratio
            if (selector & RATIO and r > ratio)
                select = true;
        }
        // only checking RATIO
        else if (!(selector & (CLUSTER1|CLUSTER2)) and (selector & RATIO)) {
            double r = tn2.getdepotdist()<tn2.getdepotdist2() ?
                tn2.getdepotdist()/tn2.getdepotdist2() :
                tn2.getdepotdist2()/tn2.getdepotdist();
            if (r <= ratio)
                select = false;
        }

        if (select and ((CLUSTER1|CLUSTER2|RATIO) & selector))
            return true;

        // is unassigned node
        if (selector & UNASSIGNED and ! unassigned[i])
            return true;

        return false;
*/
}


int TrashProblem::findNearestNodeTo(int nid, int selector, int demandLimit) {
    Trashnode &tn(datanodes[nid]);
    int nn = -1;        // init to not found
    double dist = -1;   // dist to nn

    for (int i=0; i<datanodes.size(); i++) {

        if (filterNode(tn, i, selector, demandLimit)) continue;

        double d = dMatrix[tn.getnid()][i];
        if (nn == -1 or d < dist) {
            dist = d;
            nn = i;
        }
    }
    //std::cout << "TrashProblem::findNearestNodeTo(" << nid << ", " << selector << ") = " << nn << " at dist = " << dist << std::endl;
    return nn;
}


int TrashProblem::findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int& pos) {
    Trashnode depot(v.getdepot());
    Trashnode dump(v.getdumpsite());
    int nn = -1;        // init to not found
    int loc = 0;        // position in path to insert
    double dist = -1;   // dist to nn
    double qx, qy;

    //std::cout << "TrashProblem::findNearestNodeTo(V" << depot.getnid() << ", " << selector << ")\n";

    for (int i=0; i<datanodes.size(); i++) {

        if (filterNode(depot, i, selector, demandLimit)) {
            // std::cout << "FILTERED: ";
            // datanodes[i].dump();
            continue;
        }

        double d;

        // DO NOT make this a reference|pointer or it will mess things up
        Trashnode last = depot;

        for (int j=0; j<v.size(); j++) {
//            d = distanceFromLineSegmentToPoint(
//                last.getx(), last.gety(), v[j].getx(), v[j].gety(),
//                datanodes[i].getx(), datanodes[i].gety(), &qx, &qy);
            d = datanodes[i].distanceToSegment(last,v[j]);
            if (nn == -1 or d < dist) {
                dist = d;
                loc = j;
                nn = i;
            }
            last = v[j];
        }
//        d = distanceFromLineSegmentToPoint(
//            last.getx(), last.gety(), dump.getx(), dump.gety(),
//            datanodes[i].getx(), datanodes[i].gety(), &qx, &qy);
        d = datanodes[i].distanceToSegment(last,dump);
        if (nn == -1 or d < dist) {
            dist = d;
            loc = v.size();
            nn = i;
        }
    }

    //std::cout << "TrashProblem::findNearestNodeTo(V" << depot.getnid() << ", " << selector << ") = " << nn << " at dist = " << dist << " at pos = " << loc << std::endl;

    pos = loc;
    return nn;
}


// create a CSV string that represents the solution
// this is a list of the node ids representing a vehicle route and 
// each vehicle is separated with a -1

std::string TrashProblem::solutionAsText() const {
    std::stringstream ss;;
    const std::vector<int> s = solutionAsVector();
    for (int i=0; i<s.size(); i++) {
        if (i) ss << ",";
        ss << s[i];
    }
    return ss.str();
}


// create a vector of node ids representing a solution
// this can be used to save a compute solution while other changes
// are being tried on that solution and can be used with
// buildFleetFromSolution() to reconstruct the solution

std::vector<int>  TrashProblem::solutionAsVector() const {
    std::vector<int> s;
    for (int i=0; i<fleet.size(); i++) {
        if (fleet[i].size() == 0) continue;
        for (int j=0; j<fleet[i].size(); j++) {
            s.push_back(fleet[i][j].getnid());
        }
        s.push_back(fleet[i].getdumpsite().getnid());
        s.push_back(fleet[i].getdepot().getnid());
        s.push_back(-1);
    }
    return s;
}


// reconstruct the fleet from a solution vector obtained from
// solutionAsVector(). This does some minimal checking that depot, dump
// and pickup nodes are in the correct positions
// this can also be used to construct specific solutions for testing
// algorithms

bool TrashProblem::buildFleetFromSolution(std::vector<int> solution) {
/*    unassigned = std::vector<int>(datanodes.size(), 1);

    std::vector<int>::iterator it;
    std::vector<int>::iterator it2;
    std::vector<int>::iterator start = solution.begin();

    clearFleet();

    int vid = 0;
    while ((it = std::find(start, solution.end(), -1)) != solution.end()) {
        if (*start != *(it-1) or !datanodes[*start].isdepot()) {
            // error first and last nodes must be the same depot
            std::cout << "ERROR: truck[" << vid
                      << "]: first and last nodes must be the same depot!"
                      << std::endl;
            return false;
        }
        if (!datanodes[*(it-2)].isdump()) {
            // error path[size-2] must be a dumpsite
            std::cout << "ERROR: truck[" << vid
                      << "]: path[size-2] must be a dumpsite node!"
                      << std::endl;
            return false;
        }

        Trashnode& depot(datanodes[*start]);
        Trashnode& dump(datanodes[*(it-2)]);
        unassigned[dump.getnid()] = 0;

        Vehicle truck(depot, dump);
        for (it2=start+1; it2<it-2; it2++) {
            if (*it2 < 0 or *it2 > datanodes.size()) {
                std::cout << "ERROR: truck[" << vid << "]: node: " << *it2
                          << " is NOT in range of the input problem nodes!"
                          << std::endl;
                return false;
            }
            if (datanodes[*it2].ispickup()) {
                if (unassigned[*it2] == 0) {
                    std::cout << "ERROR: truck[" << vid << "]: node: " << *it2
                              << " has already been assigned to another Truck!"
                              << std::endl;
                    return false;
                }
                unassigned[*it2] = 0;
                truck.push_back(datanodes[*it2]);
            }
            else {
                std::cout << "ERROR: truck[" << vid << "]: node: " << *it2
                          << " is not a pickup node!" << std::endl;
                return false;
            }
        }

        //truck.dump();
        fleet.push_back(truck);

        start = it+1;
        vid++;
    }
    return true;
*/
}



// NOTE: findBestFit() does check for feasibility so time windows
//       and capacity are not violated.

bool TrashProblem::findVehicleBestFit(int nid, int& vid, int& pos) {

    // track the best result found
    int vbest = -1;
    int vpos;
    double vcost;

    for (int i=0; i<fleet.size(); i++) {
        if ( fleet[i].getcargo() + datanodes[nid].getdemand() 
             > fleet[i].getmaxcapacity() ) continue;

        int tpos;
        double cost;
        Trashnode tn = datanodes[nid];

        if (fleet[i].findBestFit(tn, &tpos, &cost)) {
//std::cout << "findVehicleBestFit: nid: "<<nid<<", i: "<<i<<", tpos: "<<tpos<<", cost: "<<cost<<std::endl;
            if (vbest == -1 or cost < vcost) {
                vbest = i;
                vpos = tpos;
                vcost = cost;
            }
        }
    }

    if (vbest > -1) {
        vid = vbest;
        pos = vpos;
        return true;
    }

    return false;
}

// WARNING: this does NOT check for Time Window Feasibility !!!!!!!!!

// Initial construction of a problem solution based on nearestNeighbor
// algorithm. For each vehicle(depot) find the nearest neighbor path until
// the vehicle reaches capacity.

void TrashProblem::nearestNeighbor() {
/*
    // create a list of all pickup nodes and make them unassigned
    unassigned = std::vector<int>(datanodes.size(), 1);

    clearFleet();

    for (int i=0; i<depots.size(); i++) {

        // add this depot as the vehicles home location
        // and the associated dump
        Trashnode& depot(datanodes[depots[i]]);
        Trashnode& dump(datanodes[depot.getdumpnid()]);

        // create a vehicle and attach the depot and dump to it
        Vehicle truck(depot, dump);

        // remember the last node we inserted
        Trashnode last_node = depot;

        while (truck.getcargo() <= truck.getmaxcapacity()) {

            int nnid = findNearestNodeTo(last_node.getnid(),
                            UNASSIGNED|PICKUP|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcargo());

            // if we did not find a node we can break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            truck.push_back(datanodes[nnid]);
            last_node = datanodes[nnid];
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
*/
}


// Add all nodes to a single vehicle. Don't check for TWV or CV
// This was for testing only, not useful in real world applications

void TrashProblem::dumbConstruction() {
/*
    clearFleet();

    Trashnode& depot(datanodes[depots[0]]);
    int dnid = depot.getdumpnid();
    Trashnode& dump(datanodes[dnid]);

    Vehicle truck(depot, dump);

    for (int i=0; i<pickups.size(); i++) {
        truck.push_back(datanodes[pickups[i]]);
    }

    fleet.push_back(truck);
*/
}



// WARNING: this does NOT check for Time Window Feasibility !!!!!!!!!

//    TrashProblem::assignmentSweep
//
//    This implements Assignment Sweep construction algorithm with the
//    twist that I cluster first and identify the nearest depot (CLUSTER1)
//    and the second nearest depot (CLUSTER2) to all nodes. When nodes
//    are selected for insertions we pull from both CLUSTER1 and CLUSTER2
//    set of nodes. This might mean that the route can wander into CLUSTER2
//    territory to fill up a vehicle but it is a simple algorithm.

void TrashProblem::assignmentSweep() {
/*
    // create a list of all pickup nodes and make them unassigned
    unassigned = std::vector<int>(datanodes.size(), 1);

    clearFleet();

    for (int i=0; i<depots.size(); i++) {

        // add this depot as the vehicles home location
        // and the associated dump
        Trashnode& depot(datanodes[depots[i]]);
        Trashnode& dump(datanodes[depot.getdumpnid()]);

        // create a vehicle and attach the depot and dump to it
        Vehicle truck(depot, dump);
        //std::cout << "EMPTY TRUCK: "; truck.dump();

        int pos;
        int nid = findNearestNodeTo(truck, UNASSIGNED|PICKUP|CLUSTER1, 0, pos);
        if (nid == -1) {
            std::cout << "TrashProblem::assignmentSweep failed to find an initial node for depot: " << depots[i] << std::endl;
            continue;
        }
        truck.push_back(datanodes[nid]);
        unassigned[nid] = 0;

        int cnt = 1;
//        char buffer[100];
//        sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//        std::string str(buffer);
//        plot(str, str, truck.getpath());
        while (truck.getcargo() <= truck.getmaxcapacity()) {

//std::cout << "assignmentSweep[" << i << ',' << cnt << "] ";
//truck.dumppath();

            int nnid = findNearestNodeTo(truck,
                            UNASSIGNED|PICKUP|CLUSTER1|CLUSTER2|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcargo(),
                            pos);

            // if we did not find a node we can break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            if (pos == 0)
                truck.push_front(datanodes[nnid]);
            else if (pos == truck.size())
                truck.push_back(datanodes[nnid]);
            else 
                truck.insert(datanodes[nnid], pos);

            cnt++;
//            sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//            std::string str(buffer);
//            plot(str, str, truck.getpath());
        }

        fleet.push_back(truck);
    }
    // report unassigned nodes
    std::cout << "-------- Unassigned after TrashProblem::assignmentSweep\n";
    for (int i=0; i<pickups.size(); i++) {
        if (unassigned[pickups[i]])
            std::cout << "    " << pickups[i] << std::endl;
    }
*/
}


// WARNING: this does NOT check for Time Window Feasibility !!!!!!!!!

//    TrashProblem::assignmentSweep2
//
//    This implements Assignment Sweep construction algorithm with the
//    twist that I cluster first and identify the nearest depot (CLUSTER1)
//    and the second nearest depot (CLUSTER2) to all nodes. The construction
//    algorithm follows this pseudo code:
//    1. build routes based on only CLUSTER1 nodes
//    2. add unassigned nodes to vehicles with capacity based on CLUSTER2
//    3. add unassigned nodes to any vehicles with capacity

void TrashProblem::assignmentSweep2() {
/*    // create a list of all pickup nodes and make them unassigned
    unassigned = std::vector<int>(datanodes.size(), 1);

    clearFleet();

    for (int i=0; i<depots.size(); i++) {

        // add this depot as the vehicles home location
        // and the associated dump
        Trashnode& depot(datanodes[depots[i]]);
        Trashnode& dump(datanodes[depot.getdumpnid()]);

        // create a vehicle and attach the depot and dump to it
        Vehicle truck(depot, dump);
        //std::cout << "EMPTY TRUCK: "; truck.dump();

        int pos;
        int nid = findNearestNodeTo(truck, UNASSIGNED|PICKUP|CLUSTER1, 0, pos);
        if (nid == -1) {
            std::cout << "TrashProblem::assignmentSweep2 failed to find an initial node for depot: " << depots[i] << std::endl;
            continue;
        }
        truck.push_back(datanodes[nid]);
        unassigned[nid] = 0;

        int cnt = 1;
//        char buffer[100];
//        sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//        std::string str(buffer);
//        plot(str, str, truck.getpath());
        while (truck.getcargo() <= truck.getmaxcapacity()) {

//            std::cout << "assignmentSweep2[" << i << ',' << cnt << "] ";
//            truck.dumppath();

            int pos;
            int nnid = findNearestNodeTo(truck,
                            UNASSIGNED|PICKUP|CLUSTER1|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcargo(),
                            pos);

            // if we did not find a node we can break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            if (pos == 0)
                truck.push_front(datanodes[nnid]);
            else if (pos == truck.size())
                truck.push_back(datanodes[nnid]);
            else 
                truck.insert(datanodes[nnid], pos);

            cnt++;
//            sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//            std::string str(buffer);
//            plot(str, str, truck.getpath());
        }

        fleet.push_back(truck);
    }

    // check for unassigned nodes
    int ucnt = 0;
    for (int i=0; i<pickups.size(); i++)
        if (unassigned[pickups[i]]) ucnt++;
    if (ucnt == 0) return; // return if nothing left to do

    std::cout << ucnt << " unassigned orders after CLUSTER1 assignment" << std::endl;

    // allow vehicles to add orders from CLUSTER2
    for (int i=0; i<fleet.size(); i++) {
        Vehicle& truck = fleet[i];
        int cnt = 1;

        while (truck.getcargo() <= truck.getmaxcapacity()) {

//            std::cout << "assignmentSweep2[" << i << ',' << cnt << "] ";
//            truck.dumppath();

            int pos;
            int nnid = findNearestNodeTo(truck,
                            UNASSIGNED|PICKUP|CLUSTER2|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcargo(),
                            pos);

            // if we did not find a node we can break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            if (pos == 0)
                truck.push_front(datanodes[nnid]);
            else if (pos == truck.size())
                truck.push_back(datanodes[nnid]);
            else 
                truck.insert(datanodes[nnid], pos);

            cnt++;
//            sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//            std::string str(buffer);
//            plot(str, str, truck.getpath());
        }
    }

    // check for unassigned nodes
    ucnt = 0;
    for (int i=0; i<pickups.size(); i++)
        if (unassigned[pickups[i]]) ucnt++;
    if (ucnt == 0) return; // return if nothing left to do

    std::cout << ucnt << " unassigned orders after CLUSTER2 assignment" << std::endl;

    // allow vehicles to add any unassigned orders
    for (int i=0; i<fleet.size(); i++) {
        Vehicle& truck = fleet[i];
        int cnt = 1;

        while (truck.getcargo() <= truck.getmaxcapacity()) {

//            std::cout << "assignmentSweep2[" << i << ',' << cnt << "] ";
//            truck.dumppath();

            int pos;
            int nnid = findNearestNodeTo(truck,
                            UNASSIGNED|PICKUP|LIMITDEMAND,
                            truck.getmaxcapacity() - truck.getcargo(),
                            pos);

            // if we did not find a node we can break
            if (nnid == -1) break;

            // add node to route
            unassigned[nnid] = 0;
            if (pos == 0)
                truck.push_front(datanodes[nnid]);
            else if (pos == truck.size())
                truck.push_back(datanodes[nnid]);
            else 
                truck.insert(datanodes[nnid], pos);

            cnt++;
//            sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//            std::string str(buffer);
//            plot(str, str, truck.getpath());
        }
    }

    // report unassigned nodes
    std::cout << "-------- Unassigned after TrashProblem::assignmentSweep2\n";
    for (int i=0; i<pickups.size(); i++) {
        if (unassigned[pickups[i]])
            std::cout << "    " << pickups[i] << std::endl;
    }
*/
}



// WARNING: this does NOT check for Time Window Feasibility !!!!!!!!!

//    TrashProblem::assignmentSweep3
//
//    This implements Assignment Sweep construction algorithm with the
//    twist that I cluster first and identify the nearest depot (CLUSTER1)
//    and the second nearest depot (CLUSTER2) to all nodes. The construction
//    algorithm follows this pseudo code:
//    1. build routes based on only CLUSTER1 nodes within RATIO
//    2. add unassigned nodes based on the lowest cost to insert them

void TrashProblem::assignmentSweep3() {
    // create a list of all pickup nodes and make them unassigned
    //unassigned = std::vector<int>(datanodes.size(), 1);
    Bucket unassigned = pickups;
    Bucket problematic;
    Bucket assigned;

    std::deque<Vehicle> unusedTrucks = trucks;
    std::deque<Vehicle> usedTrucks = trucks;
    Vehicle truck;

    clearFleet();

    for (int i=0; i<trucks.size(); i++) {

        // add this depot as the vehicles home location
        // and the associated dump
        //Trashnode& depot(datanodes[depots[i]]);
        //Trashnode& dump(datanodes[depot.getdumpnid()]);

        // create a vehicle and attach the depot and dump to it
        truck=unusedTrucks[0];
        unusedTrucks.erase(unusedTrucks.begin());
        usedTrucks.push_back(truck);
        
        //std::cout << "EMPTY TRUCK: "; truck.dump();

        int pos;
        int nid = findNearestNodeTo(truck,
                UNASSIGNED|PICKUP|CLUSTER1|RATIO, 0, pos);
        if (nid == -1) {
            std::cout << "TrashProblem::assignmentSweep3 failed to find an initial node for truck: \n";
            truck.tau();
            continue;
        }
        truck.push_back(datanodes[nid]);
        unassigned.erase(datanodes[nid]);
        assigned.push_back(datanodes[nid]);

        //unassigned[nid] = 0;

        int cnt = 1;
//        char buffer[100];
//        sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//        std::string str(buffer);
//        plot(str, str, truck.getpath());
        while (truck.getcargo() <= truck.getmaxcapacity()) {

//            std::cout << "assignmentSweep2[" << i << ',' << cnt << "] ";
//            truck.dumppath();

            int pos;
            int nnid = findNearestNodeTo(truck,
                            UNASSIGNED|PICKUP|CLUSTER1|LIMITDEMAND|RATIO,
                            truck.getmaxcapacity() - truck.getcargo(),
                            pos);

            // if we did not find a node we can break
            if (nnid == -1) break;

            // TODO we need to check tw feasibility and mark the node as such
            // so findNearestNodeTo() above does not return it again

            // add node to route
            //unassigned[nnid] = 0;

            truck.insert(datanodes[nnid], pos);

            assigned.push_back(datanodes[nnid]);
            unassigned.erase(datanodes[nnid]);

            cnt++;
//            sprintf(buffer, "out/p%02d-%03d.png", i, cnt);
//            std::string str(buffer);
//            plot(str, str, truck.getpath());
        }

        fleet.push_back(truck);
    }

std::cout << "------ checking unassigned nodes ----------\n";

    // check for unassigned nodes
    /*int ucnt = 0;
    for (int i=0; i<pickups.size(); i++)
        if (unassigned[pickups[i]]) {
            ucnt++;
            datanodes[pickups[i]].dump();
        }
    if (ucnt == 0) return; // return if nothing left to do
    */
    if (unassigned.size() == 0) return; // return if nothing left to do

    std::cout << unassigned.size() << " unassigned orders after CLUSTER1|RATIO assignment" << std::endl;

    // assign remaining order based on lowest cost to insert
    // findVehicleBestFit() checks feasible()
    //for (int i=0; i<pickups.size(); i++) {
    //    if (! unassigned[pickups[i]]) continue;

    for (int i=0; unassigned.size(); i++) {
        int vid;
        int pos;
        if (! findVehicleBestFit(unassigned[0].getnid(), vid, pos)) {
            // could not find a valid insertion point
            std::cout << "assignmentSweep3: could not find a valid insertion point for node: " << unassigned[0].getnid() << std::endl;
            problematic.push_back(unassigned[0]);
            unassigned.erase(0);
            continue;
        }

        //unassigned[unassigned[i]] = 0;

        //Vehicle& truck = fleet[vid];   

        fleet[vid].insert(unassigned[0], pos);
        unassigned.erase(0);

    }

    // check for unassigned nodes
    //ucnt = 0;
    //for (int i=0; i<pickups.size(); i++)
    //    if (unassigned[pickups[i]]) ucnt++;
    if (problematic.size() == 0) return; // return if nothing left to do

    std::cout << problematic.size() << " unassigned orders after best fit assignment" << std::endl;
    problematic.dump("problematic");
}


/*


void TrashProblem::initialConstruction() {
    Bucket orders;
    Bucket incompatible;
    Trashnode lastOrder;

    for (int i=0; i<depots.size(); i++) {
        // get the depot and dump for this depots vehicle
        Trashnode& depot(datanodes[depot[i]]);
        Trashnode& dump(datanodes[depot.getdumpnid()]);

        // add nodes clustered around this depot into orders
        for (int j=0; j<pickups.size(); j++)
            if (datanodes[pickups[j]].getdepotnid() == depot.getnid())
                orders.push_back(datanodes[pickups[j]]);

        // create the truck and add the nodes to it
        Vehicle truck(depot, dump);
        makeRoute(truck, orders, incompatible, lastOrder);

        // we may not have assigned all the orders
        // and we may have incompatible orders from this truck
        // so we join these back into the orders bucket
        // so they can be applied to new trucks
        orders.join(incompatible);
        incompatible.clear();
        fleet.push_back(truck);

        truck.dump();
    }

    // now report all orders that were not assigned to any truck
    std::cout << "\n##------------- UNASSIGNED ORDERS -----------------##\n";
    orders.dump();
}

*/


/************** local route optimization ************************/

// Perform local (intra-route) optimizations on each vehicle in the fleet
// opt2opt appears to perform the best


// repeat 2-opt modifications until there is no more improvement

void TrashProblem::opt_2opt() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathTwoOpt();
    }
}


// repeat 3-opt modifications until there is no more improvement

void TrashProblem::opt_3opt() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathThreeOpt();
    }
}


// repeat Or-opt modifications until there is no more improvements

void TrashProblem::opt_or_opt() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathOrOpt();
    }
}


// repeats various path manipulations trying to optimize the path
// 1. move nodes forward
// 2. move nodes backwards
// 3. exchange nodes
// 4. invert seq

void TrashProblem::optimize() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathOptimize();
    }
}


/************** dump routines ***********************************/


// print out the distance matrix as tab separated lines like:
// irow jcol cost

void TrashProblem::dumpDmatrix() const {
    std::cout << "--------- dMatrix ------------" << std::endl;
    for (int i=0; i<dMatrix.size(); i++) {
        for (int j=0; j<dMatrix[i].size(); j++) {
            std::cout << i << "\t" << j << "\t" << dMatrix[i][j] << std::endl;
        }
    }
}


// print each vehicle in the fleet.
// This includes its stats and its path.

void TrashProblem::dumpFleet() const {
    std::cout << "--------- Fleet ------------" << std::endl;
    for (int i=0; i<fleet.size(); i++)
        fleet[i].dump();
}


// print all the datanodes that were loaded into the problem

void TrashProblem::dumpdataNodes() const {
    std::cout << "--------- Nodes ------------" << std::endl;
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}


// print all the depot nodes

void TrashProblem::dumpDepots() const {
    std::cout << "--------- Depots ------------" << std::endl;
    for (int i=0; i<depots.size(); i++)
        depots[i].dump();
}


// print all the dumpsite nodes

void TrashProblem::dumpDumps() const {
    std::cout << "--------- Dumps ------------" << std::endl;
    for (int i=0; i<dumps.size(); i++)
        dumps[i].dump();
}


// print all the pickup nodes

void TrashProblem::dumpPickups() const {
    std::cout << "--------- Pickups ------------" << std::endl;
    for (int i=0; i<pickups.size(); i++)
        pickups[i].dump();
}


// get the total duration of the solution, this includes wait time
// if early arrivel at a node

double TrashProblem::getduration() const {
    double d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getduration();
    return d;
}


// get the total cost of the solution
// cost = w1 * getduration() + w2 * getTWV() + w3 * getCV()

double TrashProblem::getcost() const {
    double d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getcost();
    return d;
}


// get the total number of TWV in the dolution

int TrashProblem::getTWV() const {
    int d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getTWV();
    return d;
}


// get the total number of CV in the solution

int TrashProblem::getCV() const {
    int d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getCV();
    return d;
}


// dump the problem and the solution

void TrashProblem::dump() const {
    dumpDepots();
    dumpDumps();
    dumpPickups();
    dumpFleet();
    std::cout << "--------- Solution ------------" << std::endl;
    std::cout << "Total path length: " << getduration() << std::endl;
    std::cout << "Total path cost: " << getcost() << std::endl;
    std::cout << "Total count of TWV: " << getTWV() << std::endl;
    std::cout << "Total count of CV: " << getCV() << std::endl;
    std::cout << "Solution: " << solutionAsText() << std::endl;
}


// dump summary of the solution

void TrashProblem::dumpSummary() const {
    std::cout << "--------- Solution ------------" << std::endl;
    std::cout << "Total path length: " << getduration() << std::endl;
    std::cout << "Total path cost: " << getcost() << std::endl;
    std::cout << "Total count of TWV: " << getTWV() << std::endl;
    std::cout << "Total count of CV: " << getCV() << std::endl;
    std::cout << "Solution: " << solutionAsText() << std::endl;
}


// create a png image of the solution and save it in "file"
// also highlight a path given as nids in vector highlight

void TrashProblem::plot( std::string file, std::string title, std::string font, std::deque<int> highlight ) {
/*    Plot<Trashnode> plot( datanodes );
    plot.setFile( file );
    plot.setTitle( title );
    plot.setFont( font );
    plot.drawInit();

    plot.drawPath(highlight, 0xffff00, 5, false);

    for (int i=0; i<fleet.size(); i++) {
        plot.drawPath(fleet[i].getpath(), plot.makeColor(i*5), 1, false);
        // printf("COLOR: %3d - 0x%06x\n", i, plot.makeColor(i));
    }
    plot.drawPoints(depots, 0xff0000, 9, true);
    plot.drawPoints(dumps, 0x00ff00, 7, true);
    plot.drawPoints(pickups, 0x0000ff, 5, true);
    plot.save();
*/
}


// create a png image of the solution and save it in "file"

void TrashProblem::plot( std::string file, std::string title, std::string font ) {
/*    Plot<Trashnode> plot( datanodes );
    plot.setFile( file );
    plot.setTitle( title );
    plot.setFont( font );
    plot.drawInit();
    for (int i=0; i<fleet.size(); i++) {
        plot.drawPath(fleet[i].getpath(), plot.makeColor(i*5), 1, false);
        // printf("COLOR: %3d - 0x%06x\n", i, plot.makeColor(i));
    }
    plot.drawPoints(depots, 0xff0000, 9, true);
    plot.drawPoints(dumps, 0x00ff00, 7, true);
    plot.drawPoints(pickups, 0x0000ff, 5, true);
    plot.save();
*/
}
