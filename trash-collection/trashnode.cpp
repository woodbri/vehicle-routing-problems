
#include <iostream>
#include <sstream>

#include "trashnode.h"

bool Trashnode::isvalid() const {
    bool ret = Node::isvalid()
                and (ispickup() or isdepot() or isdump())
                and (
                    (ispickup() and demand>0) or
                    (isdepot()  and demand>0) or
                    (isdump()   and demand==0));
    if (!ret) {
        std::cout << "Trashnode::isvalid(): failed: nid: " << nid << std::endl;
        if (! Node::isvalid())
            std::cout << "                      failed: Node::isvalid()\n";
        if (! (ispickup() or isdepot() or isdump()))
            std::cout << "                      failed: (ispickup() or isdepot() or isdump())\n";
        if (ispickup() and demand<=0)
            std::cout << "                      failed: (ispickup() and demand<=0)\n";
        if (isdepot() and demand<=0)
            std::cout << "                      failed: (isdepot() and demand<=0)\n";
        if (isdump() and demand!=0)
            std::cout << "                      failed: (isdump() and demand!=0)\n";
    }
    return ret;
};


void Trashnode::dump() const {
    std::cout << nid
              << ", " << x
              << ", " << y
              << ", " << demand
              << ", " << tw_open
              << ", " << tw_close
              << ", " << service
              << ", " << ntype
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


Trashnode::Trashnode(std::string line) : Tweval() {
    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> ntype;
    buffer >> x;
    buffer >> y;
    buffer >> demand;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> service;
}



