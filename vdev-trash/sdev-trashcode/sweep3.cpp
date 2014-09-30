
#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "plot.h"
#include "sweep3.h"



// reconstruct the fleet from a solution vector obtained from
// solutionAsVector(). This does some minimal checking that depot, dump
// and pickup nodes are in the correct positions
// this can also be used to construct specific solutions for testing
// algorithms

bool Sweep3::buildFleetFromSolution(std::vector<int> solution) {
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

bool Sweep3::findVehicleBestFit(int nid, int& vid, int& pos) {

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


void Sweep3::stepOne(Vehicle &truck, Bucket &unassigned, Bucket &assigned) {
    if (not unassigned.size()) return;
//truck.dump("turck");
//unassigned.dump("unassigned");
//assigned.dump("assigned");
    Trashnode bestNode;
    UID bestPos;
    if (truck.findNearestNodeTo( unassigned, twc,  bestPos,  bestNode) ) {
        truck.insert(bestNode,bestPos);
        assigned.push_back(bestNode);
        unassigned.erase(bestNode);
        stepOne(truck, unassigned, assigned);
    }
} 






//    Sweep3::assignmentSweep3
//
//    This implements Assignment Sweep construction algorithm with the
//    twist that I cluster first and identify the nearest depot (CLUSTER1)
//    and the second nearest depot (CLUSTER2) to all nodes. The construction
//    algorithm follows this pseudo code:
//    1. build routes based on only CLUSTER1 nodes within RATIO
//    2. add unassigned nodes based on the lowest cost to insert them

void Sweep3::assignmentSweep3() {
    // create a list of all pickup nodes and make them unassigned
    //unassigned = std::vector<int>(datanodes.size(), 1);
    Bucket unassigned = pickups;
    Bucket problematic;
    Bucket assigned;

    assert(not problematic.size());
    assert(not assigned.size());

    assert(pickups == unassigned + problematic + assigned); //the invariant

    std::deque<Vehicle> unusedTrucks = trucks;
    std::deque<Vehicle> usedTrucks = trucks;


    Vehicle truck;

    clearFleet();

    //for (int i=0; i<trucks.size(); i++) {
    while (unusedTrucks.size()) {

        truck=unusedTrucks[0];
        unusedTrucks.erase(unusedTrucks.begin());
        usedTrucks.push_back(truck);


        stepOne(truck,unassigned,assigned);        
truck.dump("turck");
unassigned.dump("unassigned");
assigned.dump("assigned");
truck.plot("sweep1","sweep1",truck.getVid());
        fleet.push_back(truck);
        assert(" end"=="");
return;
        //std::cout << "EMPTY TRUCK: "; truck.dump();

        int pos;
        int nid ;//= findNearestNodeTo(truck,
                //UNASSIGNED|PICKUP|CLUSTER1|RATIO, 0, pos);
        if (nid == -1) {
            std::cout << "Sweep3::assignmentSweep3 failed to find an initial node for truck: \n";
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
            int nnid ;//= findNearestNodeTo(truck,
                       //     UNASSIGNED|PICKUP|CLUSTER1|LIMITDEMAND|RATIO,
                        //    truck.getmaxcapacity() - truck.getcargo(),
                        //    pos);

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


void Sweep3::initialConstruction() {
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

void Sweep3::opt_2opt() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathTwoOpt();
    }
}


// repeat 3-opt modifications until there is no more improvement

void Sweep3::opt_3opt() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathThreeOpt();
    }
}


// repeat Or-opt modifications until there is no more improvements

void Sweep3::opt_or_opt() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathOrOpt();
    }
}


// repeats various path manipulations trying to optimize the path
// 1. move nodes forward
// 2. move nodes backwards
// 3. exchange nodes
// 4. invert seq

void Sweep3::optimize() {
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].pathOptimize();
    }
}


/************** dump routines ***********************************/




// print each vehicle in the fleet.
// This includes its stats and its path.

//BELONGS TO SOLUTION
void Sweep3::dumpFleet() const {
    std::cout << "--------- Fleet ------------" << std::endl;
    for (int i=0; i<fleet.size(); i++)
        fleet[i].dump();
}



// get the total duration of the solution, this includes wait time
// if early arrivel at a node

double Sweep3::getduration() const {
    double d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getduration();
    return d;
}


// get the total cost of the solution
// cost = w1 * getduration() + w2 * getTWV() + w3 * getCV()

double Sweep3::getcost() const {
    double d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getcost();
    return d;
}


// get the total number of TWV in the dolution

int Sweep3::getTWV() const {
    int d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getTWV();
    return d;
}


// get the total number of CV in the solution

int Sweep3::getCV() const {
    int d = 0;
    for (int i=0; i<fleet.size(); i++)
        d += fleet[i].getCV();
    return d;
}


// dump the problem and the solution

void Sweep3::dump() const {
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

void Sweep3::dumpSummary() const {
    std::cout << "--------- Solution ------------" << std::endl;
    std::cout << "Total path length: " << getduration() << std::endl;
    std::cout << "Total path cost: " << getcost() << std::endl;
    std::cout << "Total count of TWV: " << getTWV() << std::endl;
    std::cout << "Total count of CV: " << getCV() << std::endl;
    std::cout << "Solution: " << solutionAsText() << std::endl;
}


// create a png image of the solution and save it in "file"
// also highlight a path given as nids in vector highlight
/*
void Sweep3::plot( std::string file, std::string title, std::string font, std::deque<int> highlight ) {
    Plot<Trashnode> plot( datanodes );
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
}
*/


// create a png image of the solution and save it in "file"
/*
void Sweep3::plot( std::string file, std::string title, std::string font ) {
    Plot<Trashnode> plot( datanodes );
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
}
*/
