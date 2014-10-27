/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/

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

