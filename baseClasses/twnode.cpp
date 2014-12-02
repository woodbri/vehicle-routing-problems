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
#include <string>

#include "logger.h"
#include "twnode.h"

/*!
 * \brief Check if a Twnode is valid or not.
 * For Twnode to be valid the following conditions must be true:
 * - id \> -1
 * - tw_open \< tw_close
 * - tw_open \>= 0
 * - serviceTime \>= 0
 */
bool Twnode::isValid() const {
    if (not  Node::isValid() ) return false;
    if (not (tw_open < tw_close
            and tw_open >= 0
            and serviceTime >= 0) ) return false;
    switch  (type) {
	case 0: //depot
	    if (not demand==0) return false;
	    break;
	case 1: //dump
	    if ( demand>0) return false;
	    break;
	case 2: //pickup
	    if ( demand<=0 ) return false;
	    break;
	case 3: //ending site
	    if (not demand==0) return false;
            break;
	case 4: //delivery site
	    if (demand>=0) return false;
            break;
    }
    return true;
}


/*!
 * \brief Print the contents of a Twnode object.
 */
void Twnode::dump() const {
    std::stringstream ss;
    ss.precision( 8 );
    ss << nid
       << " = " << id
       << ",\t\ttype " << type
       << ",\tx " << x
       << ",\ty " << y
       << ",\topen " << tw_open
       << ",\tclose " << tw_close
       << ",\tdemand " << demand
       << ",\tserviceT " << serviceTime
       << ",\t street:" << streetid
       << ",\t hint:" << hint;
    DLOG(INFO) << ss.str();
}


/*!
 * \brief Set the attributes of a Twnode object.
 * \param[in] _nid Value for internal node id
 * \param[in] _id Value for user node id
 * \param[in] _x Value of the x or longitude coordinate
 * \param[in] _y Value of the y or latitude coordinate
 * \param[in] _demand Value of the demand for this node
 * \param[in] _tw_open The earliest possible arrival time
 * \param[in] _tw_close The latest possible arrivial time
 * \param[in] _service The length of time to sevice this node
 *
 * It should be noted the times are normally defined by some amount of elapsed
 * time from some problem start time of 0.0. The actual problem will specify
 * what the time units are, like seconds, minutes, hours etc.
 */
void Twnode::set( int _nid, int _id, double _x, double _y, int _demand,
                  int _tw_open, int _tw_close, int _service ) {
    nid = _nid;
    id = _id;
    x = _x;
    y = _y;
    demand = _demand;
    tw_open = _tw_open;
    tw_close = _tw_close;
    serviceTime = _service;
}


/*!
 * \brief Create an new Twnode object and populate its attributes by parsing line
 * \param[in] line A string with space separated values
 *
 * The \c line should be "nid x y tw_open tw_close demand servicetime streetid"
 */
Twnode::Twnode( std::string line ) {
    std::istringstream buffer( line );
    demand = serviceTime = 0;
    streetid = -1;
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> demand;
    buffer >> serviceTime;
    buffer >> streetid;
    id = nid;
    type = ( tw_open < tw_close and tw_open >= 0 and serviceTime >= 0 ) ? 0 : -1;
}



