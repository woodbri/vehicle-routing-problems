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

#include "tweval.h"

std::vector<std::vector<double> > Tweval::TravelTime;


/*!
 * \brief Evaluate a node at the start of a path od sub-path.
 * \param[in] cargoLimit The cargo limit for the vehicle
 */
void Tweval::evaluate ( double cargoLimit ) {
    cargo = demand;
    travelTime = 0;
    arrivalTime = opens();
    totTravelTime = 0;
    totWaitTime = 0;
    totServiceTime = serviceTime;
    departureTime = arrivalTime + serviceTime;
    twvTot = cvTot = 0;
    twv = cv = false;
    dumpVisits = 0;
    switch (type) {
        case 0: // depot or starting location
            cv = cvTot = demand > cargoLimit ? 1 : 0;
            cargo = 0;
            break;
        case 1: // dump
            dumpVisits = 1;
            break;
        default:
            cv = cvTot = demand > cargoLimit ? 1 : 0;
    };
}

/*!
 * \brief Evaluate a node in the path and update path totals from predecessor
 *
 * A path is a sequence of nodes and we maintain path statistics by keeping
 * a running sum of path related variable on each node in the path. So the
 * last node in the path will always reflect the total path. This method
 * updates those path variables based on the predecssor node.
 *
 * \param[in] pred The node preceeding this node in the path.
 * \param[in] cargoLimit The cargo limit for this vehicle.
 */
void Tweval::evaluate ( const Tweval &pred, double cargoLimit ) {
    assert( Tweval::TravelTime.size() );

    // Travel Time from previous node to this node
    travelTime    = TravelTime[pred.nid][nid];

    // tot length travel from 1st node
    totTravelTime = pred.totTravelTime + travelTime;

    // Time Window Violation
    arrivalTime   = pred.departureTime + travelTime;
    twv = latearrival( arrivalTime );

    // truck arrives before node opens, so waits
    waitTime      = earlyarrival( arrivalTime ) ? opens() - arrivalTime : 0;

    totWaitTime   = pred.totWaitTime + waitTime;

    totServiceTime = pred.totServiceTime + serviceTime;
    departureTime  = arrivalTime + waitTime + serviceTime;

    // type 1 empties the truck (aka dumpSite)
    if ( type == 1 and pred.cargo >= 0 ) demand = - pred.cargo;

    dumpVisits = ( type == 1 ) ? pred.dumpVisits + 1 :  pred.dumpVisits;

    // loading truck demand>0 or unloading demand<0
    cargo = pred.cargo + demand;

    // capacity Violation
    cv = cargo > cargoLimit or cargo < 0;

    // keep a total of violations
    twvTot = ( twv ) ? pred.twvTot + 1 : pred.twvTot;
    cvTot =  ( cv ) ?  pred.cvTot + 1 : pred.cvTot;
}


/*!
 * \brief Print the Twnode
 */
void Tweval::dump() const {
    Twnode::dump();
    std::cout << std::endl;
}

/*!
 * \brief Print the Tweval attributes for the node.
 */
void Tweval::dumpeval() const  {
    std::cout << "twv=" << twv
              << ", cv=" << cv
              << ", twvTot=" << twvTot
              << ", cvTot=" << cvTot
              << ", cargo=" << cargo
              << ", travel Time=" << travelTime
              << ", arrival Time=" << arrivalTime
              << ", wait Time=" << waitTime
              << ", service Time=" << serviceTime
              << ", departure Time=" << departureTime
              << std::endl;
}


/*!
 * \brief Construct a default Twnode
 */
Tweval::Tweval(): Twnode() {
    arrivalTime = waitTime =  travelTime = 0;
    totTravelTime = totWaitTime = totServiceTime = 0;
    twvTot = cvTot = 0;
    twv = cv = false;
}


/*!
 * \brief Test if adding some delta time to its arraive causes it to violate its time window.
 */
double Tweval::deltaGeneratesTWV( double deltaTime ) const {
    return ( arrivalTime + deltaTime > closes() );
}


