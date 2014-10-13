
#include <iostream>
#include <sstream>

#include "trashnode.h"

//
bool Trashnode::isvalid() const {
    return  ispickup()? demand>0:true; 
};


void Trashnode::dumpeval() const {
    dump();
    Tweval::dumpeval();
};

Trashnode::Trashnode(std::string line) : Tweval(line) { }



 
//OLD IDEAS

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
// END OLD IDEAS

