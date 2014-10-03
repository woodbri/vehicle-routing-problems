
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
              << ",\t\ttype " << type
              << ",\tx " << x
              << ",\ty " << y
              << ",\topen " << tw_open
              << ",\tclose " << tw_close
              << ",\tdemand " << demand
              << ",\tserviceT " << serviceTime
              << ",\t street:" << streetid;
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
    demand = serviceTime = 0;
    streetid = -1;
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> demand;
    buffer >> serviceTime;
    buffer >> streetid;
    id=nid;
    type= (tw_open < tw_close and tw_open >= 0 and serviceTime >= 0)? 0:-1;
//dump();std::cout<<"\n";
}



