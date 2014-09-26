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

