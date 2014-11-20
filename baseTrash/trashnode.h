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
#ifndef TRASHNODE_H
#define TRASHNODE_H

#include "tweval.h"

class Trashnode : public Tweval {
  protected:

  public:
    // accessors
    void dumpeval() const;

    // state
    bool isDepot() const {return type==0;};
    bool isStarting() const {return type==0;};
    bool isDump() const {return type==1;};
    bool isPickup() const {return type==2;};
    bool isEnding() const {return type==3;};
    bool isValid() const;


//Constructors
    Trashnode(std::string line);
    ~Trashnode() {};
    Trashnode() : Tweval() { }; 

};

#endif
