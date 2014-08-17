#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "Problem.h"

// NON class functions for sorting

bool sortByDist(Order a, Order b)
{
    return a.dist > b.dist;
}

// Class functions

unsigned int Problem::getNodeCount() {
    return (unsigned int) N.size();
}

unsigned int Problem::getOrderCount() {
    return (unsigned int) O.size();
}

double Problem::distance(int n1, int n2) const {
    double dx = N[n2].x - N[n1].x;
    double dy = N[n2].y - N[n1].y;
    return sqrt( dx*dx + dy*dy );
}

void Problem::dump() {
    std::cout << "---- Problem -------------\n";
    std::cout << "K: " << K << std::endl;
    std::cout << "Q: " << Q << std::endl;
    std::cout << "w1: " << w1 << std::endl;
    std::cout << "w2: " << w2 << std::endl;
    std::cout << "w3: " << w3 << std::endl;
    std::cout << "extents: " << extents[0] << ", "
                             << extents[1] << ", "
                             << extents[2] << ", "
                             << extents[3] << std::endl;
    std::cout << "---- Orders --------------\n";
    for (int i=0; i<O.size(); i++)
        O[i].dump();
    std::cout << "---- Nodes  --------------\n";
    for (int i=0; i<N.size(); i++)
        N[i].dump();
    std::cout << std::endl;
}


void Problem::loadProblem(char *infile)
{
    std::ifstream in( infile );
    std::string line;

    // read header line
    std::getline(in, line);
    std::istringstream buffer( line );
    buffer >> K;
    buffer >> Q;

    // initialize the extents
    extents[0] = std::numeric_limits<double>::max();
    extents[1] = std::numeric_limits<double>::max();
    extents[2] = std::numeric_limits<double>::min();
    extents[3] = std::numeric_limits<double>::min();


    // read the nodes
    while ( getline(in, line) ) {
        Node node;
        std::istringstream buffer( line );
        buffer >> node.nid;
        buffer >> node.x;
        buffer >> node.y;
        buffer >> node.demand;
        buffer >> node.tw_open;
        buffer >> node.tw_close;
        buffer >> node.service;
        buffer >> node.pid;
        buffer >> node.did;

        // compute the extents as we load the data for plotting
        if (node.x < extents[0]) extents[0] = node.x;
        if (node.y < extents[1]) extents[1] = node.y;
        if (node.x > extents[2]) extents[2] = node.x;
        if (node.y > extents[3]) extents[3] = node.y;

        N.push_back(node);

        if (node.nid == 0)
            DepotClose = node.tw_close;
    }
    in.close();

    // add a small buffer around the extents
    extents[0] -= (extents[2] - extents[0]) * 0.02;
    extents[2] += (extents[2] - extents[0]) * 0.02;
    extents[1] -= (extents[3] - extents[1]) * 0.02;
    extents[3] += (extents[3] - extents[1]) * 0.02;

    // make orders from the nodes
    makeOrders();

    // sort the orders
    sort(O.begin(), O.end(), sortByDist);

    calcAvgTWLen();
}


void Problem::makeOrders ()
{
    if (getNodeCount() == 0 || ((getNodeCount()-1)%2 != 0)) {
        std::string errmsg = "Problem::makeOrders - Nodes have not be correctly loaded.";
        throw std::runtime_error(errmsg);
    }

    O.reserve( (getNodeCount()-1)/2+1 );

    int oid = 0;

    // add the depot to the order list
    Order order;
    order.oid = oid++;
    order.pid = 0;
    order.did = 0;
    order.dist = 0.0;
    order.dist2 = 0.0;
    O.push_back(order);

    // for each pickup, get its delivery and create an order
    for (int i=1; i<getNodeCount(); i++) {
        if (N[i].pid == 0) {
            Order order;
            order.oid = oid++;
            order.pid = i;
            order.did = N[i].did;
            order.dist = distance(0, i);
            order.dist2 = distance(order.did, 0);
            O.push_back(order);
        }
    }
}


void Problem::calcAvgTWLen() {
    // get the average time window length
    atwl = 0;
    for (int i=0; i<N.size(); i++)
        atwl += (N[i].tw_close - N[i].tw_open);
    atwl /= N.size();
};

