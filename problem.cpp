#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "problem.h"



unsigned int Problem::getNodeCount() {
    return Nodes.size();
}


unsigned int Problem::getVehicleCount() {
    return Vehicles.size();
}


unsigned int Problem::getDumpCount() {
    return Dumps.size();
}


unsigned int Problem::getPickupCount() {
    return Pickups.size();
}


double Problem::distance(int n1, int n2) const {
    return Nodes[n1].distance(Nodes[n2]);
}


void Problem::setNodeDistances(Node& n) {
    double dist;
    int nid = -1;

    if (n.isvehicle()) {
        n.setvehicledist(n.getnid(), 0.0);
        for (int i=0; i<Dumps.size(); i++) {
            double d = n.distance(Nodes[Dumps[i]]);
            if (nid == -1 or d < dist) {
                dist = d;
                nid = Dumps[i];
            }
        }
        n.setdumpdist(nid, dist);
    }
    else if (n.isdump()) {
        n.setdumpdist(n.getnid(), 0.0);
        for (int i=0; i<Vehicles.size(); i++) {
            double d = n.distance(Nodes[Vehicles[i]]);
            if (nid == -1 or d < dist) {
                dist = d;
                nid = Vehicles[i];
            }
        }
        n.setvehicledist(nid, dist);
    }
    else if (n.ispickup()) {
        for (int i=0; i<Dumps.size(); i++) {
            double d = n.distance(Nodes[Dumps[i]]);
            if (nid == -1 or d < dist) {
                dist = d;
                nid = Dumps[i];
            }
        }
        n.setdumpdist(nid, dist);

        nid = -1;
        for (int i=0; i<Vehicles.size(); i++) {
            double d = n.distance(Nodes[Vehicles[i]]);
            if (nid == -1 or d < dist) {
                dist = d;
                nid = Vehicles[i];
            }
        }
        n.setvehicledist(nid, dist);
    }
}

void Problem::loadProblem(char *infile)
{
    std::ifstream in( infile );
    std::string line;

    // initialize the extents
    extents[0] = std::numeric_limits<double>::max();
    extents[1] = std::numeric_limits<double>::max();
    extents[2] = std::numeric_limits<double>::min();
    extents[3] = std::numeric_limits<double>::min();


    // read the nodes
    while ( std::getline(in, line) ) {
        // skip comment lines
        if (line[0] == '#') continue;

        // create node from line on file
        Node node(line);
        node.checkIntegrity();

        // compute the extents as we load the data for plotting
        if (node.getx() < extents[0]) extents[0] = node.getx();
        if (node.gety() < extents[1]) extents[1] = node.gety();
        if (node.getx() > extents[2]) extents[2] = node.getx();
        if (node.gety() > extents[3]) extents[3] = node.gety();

        Nodes.push_back(node);

        if (node.ispickup())
            Pickups.push_back(node.getnid());
        else if (node.isvehicle())
            Vehicles.push_back(node.getnid());
        else if (node.isdump())
            Dumps.push_back(node.getnid());
    }

    in.close();

    // add a small buffer around the extents
    extents[0] -= (extents[2] - extents[0]) * 0.02;
    extents[2] += (extents[2] - extents[0]) * 0.02;
    extents[1] -= (extents[3] - extents[1]) * 0.02;
    extents[3] += (extents[3] - extents[1]) * 0.02;

    for (int i=0; i<getNodeCount(); i++)
        setNodeDistances(Nodes[i]);
}


void Problem::dumpVehicles() {
    std::cout << "---- Vehicles --------------\n";
    for (int i=0; i<Vehicles.size(); i++)
        Nodes[i].dump();
}


void Problem::dumpDumps() {
    std::cout << "---- Dumps --------------\n";
    for (int i=0; i<Dumps.size(); i++)
        Nodes[i].dump();
}


void Problem::dumpPickups() {
    std::cout << "---- Pickups --------------\n";
    for (int i=0; i<Pickups.size(); i++)
        Nodes[i].dump();
}


void Problem::dump() {
    std::cout << "---- Problem -------------\n";
    std::cout << "extents: " << extents[0] << ", "
                             << extents[1] << ", "
                             << extents[2] << ", "
                             << extents[3] << std::endl;
    dumpVehicles();
    dumpDumps();
    dumpPickups();

    std::cout << std::endl;
}


