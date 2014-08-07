
#include <vector>
#include <sstream>
#include <iostream>
#include "node.h"




Node::Node(std::string line) {
        std::istringstream buffer( line );
        buffer >> nid;
        buffer >> ntype;
        buffer >> x;
        buffer >> y;
        buffer >> demand;
        buffer >> tw_open;
        buffer >> tw_close;
        buffer >> service;

        vehicledist = 0.0;
        vehiclenid = -1;
        dumpdist = 0.0;
        dumpnid = -1;
}

bool Node::checkIntegrity() {
    bool ok = true;
    if (ntype < 0 or ntype > 2) {
        std::cout << "Node[" << nid << "]: invalid type: " << ntype << std::endl;
        ok = false;
    }
    if (demand <= 0 and ntype != 1 ) {
        std::cout << "Node[" << nid << "]: invalid demand: " << demand << std::endl;
        ok = false;
    }
    if (tw_open < 0 or tw_close < tw_open) {
        std::cout << "Node[" << nid << "]: invalid values for tw_open and/or tw_close (0 < tw_open[" << tw_open << "] < tw_close[" << tw_close << "])" << std::endl;
        ok = false;
    }
    if (service < 0) {
        std::cout << "Node[" << nid << "]: invalid service time" << std::endl;
        ok = false;
    }
    return ok;
}

void Node::setvehicledist(int nid, double dist) {
    vehicledist = dist;
    vehiclenid = nid;
}


void Node::setdumpdist(int nid, double dist) {
    dumpdist = dist;
    dumpnid = nid;
}


void Node::dump()const {
    std::cout << "Node: "
              << nid << ", "
              << ntype << ", "
              << x << ", "
              << y << ", "
              << demand << ", "
              << tw_open << ", "
              << tw_close << ", "
              << service << ", "
              << vehicledist << ", "
              << vehiclenid << ", "
              << dumpdist << ", "
              << dumpnid
              << std::endl;
}


