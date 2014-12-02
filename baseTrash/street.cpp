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
#include <deque>
#include <limits>

#include "logger.h"
#include "twpath.h"
#include "street.h"



void Street::dumpeval() const {
    DLOG( INFO ) << "Street id: " << sid
                 << "\tRequired capacity: " << _reqCapacity
                 << "\tRequired time: " << _reqTime;
    path.dumpeval();
}



void Street::dump() const {
    std::stringstream sidStr;
    sidStr << "street ID " << sid << ":\t";
    path.dump( sidStr.str() );
}

void Street::dumpid() const {
    std::stringstream sidStr;
    sidStr << "street ID " << sid << ":\t";
    path.dumpid( sidStr.str() );
}


void Street::dump( const std::string &title ) const { path.dump( title ); }




bool Street::e_push_back( const Trashnode &node ) {
    assert ( node.getnid() >= 0 );

    if ( sid == -1 or node.streetId() != sid ) return false;

    path.push_back( node );
    path.evaluate( MAX() );
    evalLast();
    return true;
}


bool Street::e_push_front( const Trashnode &node ) {
    assert ( node.getnid() >= 0 );

    if ( sid == -1 or node.streetId() != sid ) return false;

    path.push_front( node  );
    path.evaluate( 0, MAX() );
    evalLast();
    return true;
}


int Street::getBestPos( const Trashnode &node ) {
    double bestDist = MAX();
    int i = 0;

    while ( i < size() and node.distance( path[i] ) < bestDist ) {
        bestDist = node.distance( path[i] ); i++;
    }

    return --i;
}


bool Street::insert( const Trashnode &node ) {
    if ( sid == -1 or node.streetId() != sid ) return false;

    if ( not size() ) return e_push_back( node );

    int at = getBestPos( node );
    path.insert( node, at );
    return true;
}

bool Street::e_insert( const Trashnode &node ) {
    if ( not insert( node ) ) return false;

    path.evaluate( MAX() );
    evalLast();
    return true;
}

// e_ insert inserts in order of distances the containers of the street
// check the order
int Street::e_insert( Bucket &unassigned, Bucket &assigned ) {
    Bucket nodes = unassigned;
    unassigned.clear();
    assigned.clear();
    Trashnode node;

    while ( nodes.size() ) {
        node = nodes[0];

        if ( insert( node ) == false ) unassigned.push_back( node );
        else assigned.push_back( node );

        nodes.pop_front();
    };

    path.evaluate( 0, MAX() );
    evalLast();
    return size();
}


int Street::e_insert( const Bucket &containers ) {
    Bucket unassigned = containers;
    Bucket assigned;
    return e_insert( unassigned , assigned );
}


/*

bool Street::remove(const Trashnode &node ) {
    if (not path.in( node );
    int at;
    if ( at=path.pos( node ) <0) return false;
    path.remove(at);
    path.evaluate(at,MAX());
    return true;
}


bool Street::moverange( int rangefrom, int rangeto, int destbefore ) {
    E_Ret ret = path.e_move(rangefrom, rangeto, destbefore, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::movereverse( int rangefrom, int rangeto, int destbefore ) {
    E_Ret ret = path.e_movereverse(rangefrom, rangeto, destbefore, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::reverse( int rangefrom, int rangeto ) {
    E_Ret ret = path.e_reverse(rangefrom, rangeto, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::move( int fromi, int toj ) {
    E_Ret ret = path.e_move(fromi, toj, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::swap( const int& i, const int& j ) {
    E_Ret ret = path.e_swap(i, j, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::swap(Street& v2, const int& i1, const int& i2) {
    E_Ret ret = path.e_swap(i1, getmaxcapacity(),
                    v2.getvpath(), i2, v2.getmaxcapacity());
    if (ret == OK) {
        evalLast();
        v2.evalLast();
    }
    else if (ret == INVALID) return false;
    return true;
}


void Street::restorePath(Twpath<Trashnode> oldpath) {
    path = oldpath;
    evalLast();
}
*/

void Street::evalLast() {
    Trashnode last = path[path.size() - 1];
    _reqCapacity = last.getCargo();
    _reqTime = last.getTotTime();
}

/**************************************PLOT************************************/
void Street::plot( std::string file, std::string title ) {
    Twpath<Trashnode> trace = path;
    std::stringstream sidStr;
    sidStr << sid;
    std::string extra = file + " street " + sidStr.str() ;

    Plot<Trashnode> graph( trace );
    graph.setFile( file + ".png" );
    graph.setTitle( title + extra );
    graph.drawInit();

    for ( int i = 0; i < trace.size(); i++ ) {
        if ( trace[i].isPickup() )  {
            graph.drawPoint( trace[i], 0x0000ff, 9, true );
        }
        else if ( trace[i].isDepot() ) {
            graph.drawPoint( trace[i], 0x00ff00, 5, true );
        }
        else  {
            graph.drawPoint( trace[i], 0xff0000, 7, true );
        }
    }

    plot( graph );
    graph.save();
}

void Street::plot( Plot<Trashnode> graph ) {
    graph.drawPath( path, graph.makeColor( sid * 10 ), 1, true );
}


