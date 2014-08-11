
#include <iostream>
#include <sstream>

#include "node.h"

void Node::dump() const {
    std::cout << nid
              << ", " << x
              << ", " << y
              << std::endl;
}


Node::Node(std::string line) {
    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
}
