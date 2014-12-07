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
#ifndef TRASHPROB_H
#define TRASHPROB_H

#include <string>
#include <sstream>

#include "vrptools.h"
#include "prob_trash.h"

class TrashProb : public Prob_trash {

  public:

    TrashProb() : Prob_trash() {};

    void addContainers( container_t *containers, int count );
    void addOtherlocs( otherloc_t *otherlocs, int count );
    bool checkNodesOk();
    void addTtimes( ttime_t *ttimes, int count );
    void addVehicles( vehicle_t *vehicles, int count );

    bool isValid() const;
    std::string whatIsWrong() const;

};

#endif
