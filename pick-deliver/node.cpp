#include <math.h>
#include <iostream>
#include <sstream>
#include "node.h"

Node::Node(std::string line) {
        std::istringstream buffer( line );
        buffer >> nid;
        buffer >> x;
        buffer >> y;
}

int Node::quadrant() const {
   if (x>0)
      if (y>0) return 1;
      else return 2;
   else
      if (y>0) return 4;
      else return 3;
}

int Node::quadrant(const Node &from) const {
   if (x>from.x)
      if (y>from.y) return 1;
      else return 2;
   else
      if (y>from.y) return 4;
      else return 3;
}

double Node::distance(const Node &n2) const {
         double dx = n2.x - x;
         double dy = n2.y - y;
         return sqrt( dx*dx + dy*dy );
};


void Node::dump()const {
    std::cout << "N#" << nid << ", "
              << "("<< x << ", " << y << ") ";
}

void Node::debugdump()const {
    std::cout << "nid=" << nid << ", "
              << "("<< x << ", " << y << ") ";
}

