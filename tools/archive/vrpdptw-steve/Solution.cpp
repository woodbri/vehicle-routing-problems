
#include "Route.h"
#include "Solution.h"

// NON class functions for sorting

bool sortByOid(Order a, Order b)
{
    return a.oid < b.oid;
}

// Class functions

void Solution::sequentialConstruction() {
    // std::cout << "Enter Problem::sequentialConstruction\n";
    int M = 0;
    R.clear();
    int numUnassigned = P.O.size() - 1;
    while (numUnassigned > 0) {
        Route r(P);
        r.rid = M;

        for (int i=0; i<P.O.size(); i++) {
            if (P.O[i].oid == 0) continue;    // don't add the depot
            if (mapOtoR[P.O[i].oid] != -1) continue; // search unassigned orders

            r.addOrder(P.O[i]);
            mapOtoR[P.O[i].oid] = r.rid;
            r.hillClimbOpt();

            // if route is not feasible
            if (r.orders.size() > 1 && (r.TWV > 0 || r.CV > 0)) {
                r.removeOrder(P.O[i]);
                mapOtoR[P.O[i].oid] = -1;
            }
            // else if it is feasible
            else {
                mapOtoR[P.O[i].oid] = r.rid;
                numUnassigned--;
            }
        }
        R.push_back(r);
        M++;
    }

    sort(P.O.begin(), P.O.end(), sortByOid);

    //std::cout << "Exit Problem::sequentialConstruction\n";
}


void Solution::initialConstruction() {
    int M = 0;
    R.clear();

    for (int i=0; i<P.O.size(); i++) {
        if (P.O[i].oid == 0) continue;    // don't add the depot
        Route r(P);
        r.rid = M++;
        r.addOrder(P.O[i]);
        mapOtoR[P.O[i].oid] = r.rid;
        R.push_back(r);
    }

    sort(P.O.begin(), P.O.end(), sortByOid);
}

void Solution::computeCosts() {
    totalCost = 0.0;
    totalDistance = 0.0;
    for (int i=0; i<R.size(); i++) {
        totalCost += R[i].getCost();
        totalDistance += R[i].D;
    }
}

double Solution::getCost() {
    return totalCost;
}

double Solution::getDistance() {
    return totalDistance;
}


void Solution::dump() {
    computeCosts();
    std::cout << "Solution: totalDistance: " << totalDistance
              << ", totalCost: " << totalCost
              << std::endl << "Routes:" << std::endl;
    for (int i=0; i<R.size(); i++)
        R[i].dump();
    std::cout << "mapOtoR: ";
    for (int i=0; i<mapOtoR.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << mapOtoR[i];
    }
    std::cout << std::endl;
}


double Solution::getAverageRouteDurationLength() {
    double len = 0.0;
    int n = 0;
    for (int i=0; i<R.size(); i++) {
        if (R[i].path.size() == 0) continue;
        if (R[i].updated) R[i].update();
        len += R[i].D;
        n++;
    }
    if (n == 0) {
        std::string errmsg = "Solution.getAverageRouteDurationLength: There do not appear to be any routes!";
        throw std::runtime_error(errmsg);
    }
    return len/n;
}
