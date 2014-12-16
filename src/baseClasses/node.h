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
#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <string>
#include <limits>

/*! \class Node
 * \brief The Node class defines a point in 2D space with an id.
 *
 * A Node is a point that defines a location in 2D space. It maintains
 * a user \c id and an internal \c nid along with its \c x, \c y location.
 * This is the base object that things like depots, customer locations, etc.
 * are built upon.
 *
 */

class Node {
  protected:
    int nid;    ///< internal node number
    int id;     ///< user supplied node number
    double x;   ///< x or longitude of the node's location
    double y;   ///< y or latitude of the node's location
    std::string hint;

  public:
    // accessors
    int getnid() const { return nid; };
    int getid() const { return id; };
    double getx() const { return x; };
    double gety() const { return y; };
    std::string getHint() const { return hint; };

    double distance( const Node &n ) const ;


    // state
    bool isLatLon() const { return ((x<180) and (x>-180)) and ((y<180) and (y>-180)) ;}
    bool isValid() const { return  id > -1; };
    bool isSamePos( const Node &n ) const { return distance( n ) == 0; };
    bool isSamePos( const Node &n, double tol ) const { return distance( n ) < tol; };
    bool hasHint() const {return not ( hint == "" );};

    // mutators
    void set( int _nid, double _x, double _y );
    void setnid( int _nid ) { nid = _nid; };
    void setid( int _id ) { id = _id; };
    void setx( double _x ) { x = _x; };
    void sety( double _y ) {y = _y; };
    void setHint( const std::string &_hint ) {hint = _hint; };

    // operators
    bool operator<( const Node &n ) const { return nid < n.nid; };
    bool operator==( const Node &n ) const { return nid == n.nid && x == n.x && y == n.y; };
    bool operator!=( const Node &n ) const { return ! ( *this == n ); };
    bool operator>( const Node &n ) const { return nid > n.nid; };

    // vector operators
    Node operator+( const Node &v ) const;
    Node operator-( const Node &v ) const;
    Node operator*( double f ) const;
    double dotProduct( const Node &p ) const ;
    double length() const ;
    double gradient( const Node &p ) const ;
    double distanceTo( const Node &p ) const ;
    double distanceToSquared( const Node &p ) const ;
    Node unit() const ;
    double distanceToSegment( const Node &v, const Node &w ) const ;
    double distanceToSegment( const Node &v, const Node &w, Node &q ) const;
    double distanceToSegment( double, double, double, double, double &,
                              double & ) const;

    // dump
    void dump() const;

    // constructors

    Node() ;
    Node( int _nid, double _x, double _y ) ;
    Node( int _nid, int id, double _x, double _y ) ;
    Node( double _x, double _y ) ;
    Node( const std::string line );

    ~Node() {};

};



#endif
