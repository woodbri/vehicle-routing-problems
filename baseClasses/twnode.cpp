
#include <iostream>
#include <sstream>
#include <string>

#include "twnode.h"

bool Twnode::isvalid() const {
    return Node::isvalid() and tw_open < tw_close
                           and tw_open >= 0
                           and serviceTime >= 0;
}


void Twnode::dump() const {
    std::cout.precision(8);
    std::cout << nid
              << " = " << id
              << ",\t\t " << type
              << ",\t " << x
              << ",\t " << y
              << ",\t " << demand
              << ",\t " << tw_open
              << ",\t " << tw_close
              << ",\t " << serviceTime;
}


void Twnode::set(int _nid, int _id, double _x, double _y, int _demand,
                 int _tw_open, int _tw_close, int _service) {
    nid = _nid;
    id = _id;
    x = _x;
    y = _y;
    demand = _demand;
    tw_open = _tw_open;
    tw_close = _tw_close;
    serviceTime = _service;
}


Twnode::Twnode(std::string line) {
    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> type;
    buffer >> x;
    buffer >> y;
    buffer >> demand;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> serviceTime;
    id=nid;
}



