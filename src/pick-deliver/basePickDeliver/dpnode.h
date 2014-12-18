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
#ifndef DPNODE_H
#define DPNODE_H

#include <string>
#include "tweval.h"

class Dpnode: public Tweval {
private:
    //to know the other node in the order
    int oid;
    int did;
    int pid;
public:
    bool ispickup() const {return demand>0;}
    bool isdelivery() const {return demand<0;}
    bool isdepot() const {return  hasnogoods();}

    void dumpeval() const;
    void dump() const ;
/*accessors*/
    int getdid() const {return  did;};
    int getpid() const {return  pid;};
    int getoid() const {return oid;};
/* mutators */        
    void setoid(int _oid)  {oid=_oid;};
    bool operator== (const Dpnode& other) const{ return getnid()==other.getnid();};
    bool operator< (const Dpnode& other) const{ return getnid()< other.getnid();};

   Dpnode(std::string line);

   Dpnode():Tweval(){};
   ~Dpnode(){};

};    

#endif
