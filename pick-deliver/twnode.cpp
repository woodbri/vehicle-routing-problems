#include <sstream>
#include "node.h"
#include "twnode.h"

Twnode::Twnode(std::string line) {
        std::istringstream buffer( line );
        buffer >> nid;
        buffer >> x;
        buffer >> y;
        buffer >> demand;
        buffer >> tw_open;
        buffer >> tw_close;
        buffer >> service;
}


bool Twnode::checkintegrity() const {
     return Node::isvalid() and tw_open < tw_close
            and tw_open >= 0 and service >= 0;
}



void Twnode::dump()const {
    std::cout << nid <<"= ("<<x<<", "<<y<<")";
};

void Twnode::debugdump()const {
    std::cout << nid <<", ("
              <<x<<", "
              <<y<<", "
              << demand << ", "
              << tw_open << ", "
              << tw_close << ", "
              << service<<")";
}

