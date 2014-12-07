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

#ifdef LOG
#include "logger.h"
#endif

#include "node.h"


/*!
 * \brief Compute the Haversine spherical distance between two nodes.
 *
 * Haversine spherical distance between two nodes with lat/lon values when
 * the nodes x,y is loaded with longitude,latitude values.
 *
 */
double Node::distance( const Node &n ) const {
    // Haversine sphereical distance for lat/lon values
    const double deg2rad = 3.14159265358979323846 / 180.0;
    const double rad2deg = 180.0 / 3.14159265358979323846;
    const double radius = 6367000; // Earth radius 6367 Km in meters
    double dlon = ( n.x - x ) * deg2rad;
    double dlat = ( n.y - y ) * deg2rad;
    double a = pow( sin( dlat / 2.0 ), 2 ) + cos( y ) * cos( n.y ) *
               pow( sin( dlon / 2.0 ), 2 );
    double c = 2.0 * atan2( sqrt( a ), sqrt( 1.0 - a ) );
    double dist = radius * c;
    return dist;

    // Simple Euclidean distance in x-y plane
    //double dx = n.x - x;
    //double dy = n.y - y;
    //return sqrt( dx*dx + dy*dy );
};


/*!
 * \brief Set attributes for this node.
 */
void Node::set( int _nid, double _x, double _y ) {
    id = nid = _nid;
    x = _x;
    y = _y;
};

/*!
 * \brief Print the contents of this node.
 */
void Node::dump() const {
    #ifdef LOG
    DLOG( INFO ) << nid
                 << ", " << x
                 << ", " << y;
    #endif
};

// Vector Operations

/*!  \brief Create a new Node by performing vector addition.  */
Node  Node::operator+( const Node &v ) const { return Node( x + v.x, y + v.y ); };

/*!
 * \brief Create a new Node by performing vector subtraction.
 */
Node  Node::operator-( const Node &v ) const { return Node( x - v.x, y - v.y ); };

/*!
 * \brief Create a new Node by scaling and existing node by a factor \c f.
 */
Node  Node::operator*( double f ) const { return Node( x * f, y * f ); };

/*!
 * \brief Compute the vector dot product between two Nodes where the location is considered a vector.
 */
double Node::dotProduct( const Node &p ) const { return x * p.x + y * p.y; };

/*!
 * \brief Compute the Euclidean length of a vector
 */
double Node::length() const { return sqrt( x * x + y * y ); };

/*!
 * \brief Compute the gradient or slope of a vector defined by the vector n->p
 * \bug This is not safe as it can generate a divide by zero
 * \todo This needs to be fixed to avoid divide by zero errors
 */
double Node::gradient( const Node &p ) const {
    double deltaY = p.y - y;
    double deltaX = p.x - x;

    if ( deltaX == 0 )
        if ( deltaY >= 0 ) return std::numeric_limits<double>::max();
        else return -std::numeric_limits<double>::max();
    else return  deltaY / deltaX;
};

/*!
 * \brief Compute the Euclidean distance between to Nodes.
 * \sa Node::length, Node::distance, Node::distanceToSquared
 */
double Node::distanceTo( const Node &p ) const { return sqrt( distanceToSquared( p ) ); };

/*!
 * \brief Compute the Euclidean distance squared between two Nodes.
 *
 * \sa Node::length, Node::distanceTo, Node::distance
 */
double Node::distanceToSquared( const Node &p ) const {
    const double dX = p.x - x;
    const double dY = p.y - y;

    return dX * dX + dY * dY;
};

/*!
 * \brief Create a new node where the location is a unit vector of the reference Node.
 */
Node Node::unit() const {
    double scale = 0.0;
    double len = length();

    if ( len != 0.0 )
        scale = 1.0 / len;

    return ( *this ) * scale;
};

/*!
 * \brief Compute the shortest distance from a Node to a line segment from Node \c v to Node \c w
 */
double Node::distanceToSegment( const Node &v, const Node &w ) const {
    Node q;
    return distanceToSegment( v, w, q );
};

/*!
 * \brief Compute the shortest distance and Node \c q from a Node to a line segment from Node \c v to Node \c w
 */
double Node::distanceToSegment( const Node &v, const Node &w, Node &q ) const {

    // i.e. |w-v|^2 ... avoid a sqrt
    double distSq = v.distanceToSquared( w );

    if ( distSq == 0.0 ) { // v == w case
        q = v;
        return distanceTo( v );
    }

    // consider the line extending the segment, parameterized as v + t (w - v)
    // we find projection of point p onto the line
    // it falls where t = [(p-v) . (w-v)] / |w-v|^2

    double t = ( ( *this ) - v ).dotProduct( w - v ) / distSq;

    if ( t < 0.0 ) { // beyond the v end of the segment
        q = v;
        return distanceTo( v );
    }

    if ( t > 1.0 ) { // beyond the w end of the segment
        q = w;
        return distanceTo( w );
    }

    // projection falls on the segment
    Node projection = v + ( ( w - v ) * t );

    q = projection;

    return distanceTo( projection );
}

/*!
 * \brief Compute the shortest distance and Node \c q to a line segment defined by its x,y end points and return x,y position on the segment of the closest point.
 */
double Node::distanceToSegment( double segmentX1, double segmentY1,
                                double segmentX2, double segmentY2, double &qX, double &qY ) const {
    Node q;

    double distance = distanceToSegment( Node( segmentX1, segmentY1 ),
                                         Node( segmentX2, segmentY2 ), q );

    qX = q.x;
    qY = q.y;

    return distance;
}


// Constructors

/*!
 * \brief Construct a new Node that needs the user to set its attributes.
 */
Node::Node() {
    id = nid = -1;
    x = 0.0;
    y = 0.0;
    hint = "";
};

/*!
 * \brief Construct a new Node and assign it \c x and \c y values.
 */
Node::Node( double _x, double _y ) {
    id = nid = -1;
    x = _x;
    y = _y;
    hint = "";
};

/*!
 * \brief Construct a new Node and assign it \c nid, \c x and \c y values.
 */
Node::Node( int _nid, double _x, double _y ) {
    id = -1;
    nid = _nid;
    x = _x;
    y = _y;
    hint = "";
};

/*!
 * \brief Construct a new Node and assign it the associated values.
 */
Node::Node( int _nid, int _id , double _x, double _y ) {
    id = _id;
    nid = _nid;
    x = _x;
    y = _y;
    hint = "";
};

/*!
 * \brief Create a new Node by parsing a string.
 */
Node::Node( std::string line ) {
    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    id = nid;
    hint = "";
}
