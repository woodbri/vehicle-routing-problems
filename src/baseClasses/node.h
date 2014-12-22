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
#ifndef SRC_BASECLASSES_NODE_H_
#define SRC_BASECLASSES_NODE_H_

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
 private:
    int nid_;    ///< internal node number
    int id_;     ///< user supplied node number
    double x_;   ///< x or longitude of the node's location
    double y_;   ///< y or latitude of the node's location
    std::string hint_;  ///< orsrm's hint

 public:
    // accessors
    int nid() const {return nid_;}
    int id() const {return id_;}
    double x() const {return x_;}
    double y() const {return y_;}
    std::string hint() const {return hint_;}

    double distance(const Node &n) const;


    // state
    bool isLatLon() const { return (x_ < 180) && (x_ > -180)
                                && (y_ < 180) && (y_ > -180);}
    bool isValid() const { return  id_ > -1;}
    bool isSamePos(const Node &other) const { return distance(other) == 0; }
    bool isSamePos(const Node &other, double tolerance) const {return distance(other) < tolerance;}
    bool hasHint() const {return !( hint_ == "" );}

    // mutators
    void set(int nid, double x, double y);
    void setnid(int _nid) {nid_ = nid;}
    void setid(int id) {id_ = id; }
    void setx(double x) {x_ = x; }
    void sety(double y) {y_ = y; }
    void setHint(const std::string &hint) {hint_ = hint;}

    // operators
    bool operator<( const Node &n) const { return nid_ < n.nid_; }
    bool operator==(const Node &n) const { return nid_ == n.nid_
                                                  && x_ == n.x_ && y_ == n.y_; }
    bool operator!=(const Node &n) const { return !( *this == n ); }
    bool operator>(const Node &n) const { return nid_ > n.nid_; }

    // vector operators
    Node operator+(const Node &v) const;
    Node operator-(const Node &v) const;
    Node operator*(double f) const;
    double dotProduct(const Node &p) const;
    double length() const;
    double gradient(const Node &pi) const;
    double distanceTo(const Node &p) const;
    double distanceToSquared(const Node &p) const;
    Node unit() const;
    double distanceToSegment(const Node &v, const Node &w) const;
    double distanceToSegment(const Node &v, const Node &w, Node &q) const;
    double distanceToSegment(double, double, double, double, double &,
                              double &) const;

    // dump
    void dump() const;

    // constructors

    Node();
    Node(int nid, double x, double y);
    Node(int nid, int id, double x, double y);
    Node(double x, double y);
    explicit Node(const std::string &line);

    ~Node() {}
};



#endif  // SRC_BASECLASSES_NODE_H_
