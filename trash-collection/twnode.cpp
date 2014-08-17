
#include <iostream>
#include <sstream>
#include <string>

#include "twnode.h"

bool Twnode::checkintegrity() const {
    return Node::isvalid() and tw_open < tw_close
                           and tw_open >= 0
                           and service >= 0;
}


void Twnode::dump() const {
    std::cout << nid
              << ", " << x
              << ", " << y
              << ", " << demand
              << ", " << tw_open
              << ", " << tw_close
              << ", " << service
              // << ", " << pid
              // << ", " << did
              << std::endl;
}


void Twnode::set(int _nid, double _x, double _y, int _demand,
                 int _tw_open, int _tw_close, int _service) {
    nid = _nid;
    x = _x;
    y = _y;
    demand = _demand;
    tw_open = _tw_open;
    tw_close = _tw_close;
    service = _service;
}


Twnode::Twnode(std::string line) {
    // TODO: extend this to support pickup and delivery pairs

    //int pid;
    //int did;

    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    buffer >> demand;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> service;
    //buffer >> pid;
    //buffer >> did;
}



