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
#include <string>
#include <iostream>
#include <sstream>

#include "dpnode.h"


    void Dpnode::dump() const {
        Twnode::dump();
        std::cout<<"\t pid="<<pid
                 <<"\t did="<<did
                 <<"\t oid="<<oid;
       if (ispickup()) std::cout<<"\tpickup:"<<demand;
       if (isdelivery()) std::cout<<"\tdelivery:"<<demand;
       std::cout<<"\n";
        }

    void Dpnode::dumpeval() const  {
        dump();
        Tweval::dumpeval();
    };

Dpnode::Dpnode(std::string line):Tweval() {

    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    buffer >> demand;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> service;
    buffer >> pid;
    buffer >> did;
    id=nid;
}

