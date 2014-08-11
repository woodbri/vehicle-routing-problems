
#include <iostream>
#include <sstream>

#include "trashnode.h"

void Trashnode::dump() const {
    std::cout << nid
              << ", " << x
              << ", " << y
              << ", " << demand
              << ", " << tw_open
              << ", " << tw_close
              << ", " << service
              << std::endl;
}


void Trashnode::setvalues(int _nid, double _x, double _y, int _demand,
                          int _tw_open, int _tw_close, int _service,
                          int _ntype) {
    nid = _nid;
    x = _x;
    y = _y;
    demand = _demand;
    tw_open = _tw_open;
    tw_close = _tw_close;
    service = _service;
    ntype = _ntype;
}


void Trashnode::setdepotdist(int _nid, double _dist, int _nid2, double _dist2) {
    depotnid = _nid;
    depotdist = _dist;
    depotnid2 = _nid2;
    depotdist2 = _dist2;
}


void Trashnode::setdumpdist(int _nid, double _dist) {
    dumpnid = _nid;
    dumpdist = _dist;
}


Trashnode::Trashnode(std::string line) {
    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    buffer >> demand;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> service;
}



