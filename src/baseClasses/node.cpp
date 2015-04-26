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

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>

#ifdef DOVRPLOG
#include "./logger.h"
#endif

#include "./node.h"


/*!
 * \brief Compute the Haversine spherical distance between two nodes.
 *
 * Haversine spherical distance between two nodes with lat/lon values when
 * the nodes x,y is loaded with longitude,latitude values.
 *
 */
double Node::haversineDistance(const Node &other) const {
  const double pi = 3.14159265358979323846;
  const double deg2rad = pi / 180.0;
  const double radius = 6367000;  // Earth radius 6367 Km in meters
  double dlon = (other.x_ - x_) * deg2rad;
  double dlat = (other.y_ - y_) * deg2rad;
  double a = pow(sin(dlat / 2.0), 2) + cos(y_) * cos(other.y_) *
             pow(sin(dlon / 2.0), 2);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double dist = radius * c;
  return dist;
}

double Node::distance(const Node &other) const {
  if (!(isLatLon() && other.isLatLon())) return distanceTo(other);

  return haversineDistance(other);
}

/*!  * \brief Set attributes for this node.  */
void Node::set(UID id, double x, double y) {
  id_ = nid_ = id;
  x_ = x;
  y_ = y;
  valid_ = true;
}

void Node::clear() {
  nid_ = 0; id_ = 0;
  x_ = 0.0; y_ = 0.0;
  hint_ = "";
  valid_ = false;
}
/*! \brief Set attributes by parsing a string.  */
void Node::set(const std::string &line) {
  clear();
  std::istringstream buffer(line);
  long int ids;
  buffer >> ids;
  buffer >> x_;
  buffer >> y_;
  hint_ = "";
  if (ids < 0) {
    valid_ = false;
  } else {
    nid_ = UID(ids);
    id_ = UID(ids);
  }
}



/*!  * \brief Print the contents of this node.  */
#ifdef DOVRPLOG
void Node::dump() const {
  DLOG(INFO) << nid_
             << ", " << x_
             << ", " << y_
             << ", " << hint_;
}
#endif

// Vector Operations

/*! \brief Create a new Node by performing vector addition.  */
Node  Node::operator+(const Node &v) const {
  return Node( x_ + v.x_, y_ + v.y_ );
}

/*! \brief Create a new Node by performing vector subtraction.  */
Node  Node::operator-(const Node &v) const {
  return Node( x_ - v.x_, y_ - v.y_ );
}

/*! \brief Create a new Node by scaling and existing node by a factor \c f.*/
Node  Node::operator*(double f) const { return Node( x_ * f, y_ * f ); }

/*! \brief Compute the vector dot product between two Nodes.*/
double Node::dotProduct( const Node &p ) const { return x_ * p.x_ + y_ * p.y_; }

/*! \brief Compute the Euclidean length of a vector */
double Node::length() const { return sqrt( x_ * x_ + y_ * y_ ); }

/*! \brief Compute the gradient or slope of a vector defined by the vector n->p
 * \bug This is not safe as it can generate a divide by zero
 * \todo This needs to be fixed to avoid divide by zero errors
 */
double Node::gradient(const Node &p) const {
  double deltaY = p.y_ - y_;
  double deltaX = p.x_ - x_;

  if (deltaX == 0) {
    if (deltaY >= 0)
      return VRP_MAX();
    else
      return VRP_MIN();
  } else {
    return  deltaY / deltaX;
  }
}

/*! \brief Compute the Euclidean distance between to Nodes.
 * \sa Node::length, Node::distance, Node::distanceToSquared
 */
double Node::distanceTo(const Node &p) const {
  return sqrt(distanceToSquared(p));
}

/*!  \brief Compute the Euclidean distance squared between two Nodes.
 *
 * \sa Node::length, Node::distanceTo, Node::distance
 */
double Node::distanceToSquared(const Node &p) const {
  const double dX = p.x_ - x_;
  const double dY = p.y_ - y_;

  return dX * dX + dY * dY;
}

/*!  \brief Calculates the unit vector of the reference Node.  */
Node Node::unit() const {
  double scale = 0.0;
  double len = length();

  if ( len != 0.0 )
    scale = 1.0 / len;

  return (*this) * scale;
}

/*! \brief Compute the shortest distance from a Node to a line segment*/
double Node::distanceToSegment(const Node &v, const Node &w) const {
  Node q;
  return distanceToSegment( v, w, q );
}

/*! \brief Compute the shortest distance from a Node to a line segment */
double Node::distanceToSegment(const Node &v, const Node &w, Node &q) const {
  // i.e. |w-v|^2 ... avoid a sqrt
  double distSq = v.distanceToSquared(w);

  if (distSq == 0.0) {  // v == w case
    q = v;
    return distanceTo(v);
  }

  // consider the line extending the segment, parameterized as v + t (w - v)
  // we find projection of point p onto the line
  // it falls where t = [(p-v) . (w-v)] / |w-v|^2

  double t = ((*this) - v).dotProduct(w - v) / distSq;

  if ( t < 0.0 ) {  // beyond the v end of the segment
    q = v;
    return distanceTo(v);
  }

  if ( t > 1.0 ) {  // beyond the w end of the segment
    q = w;
    return distanceTo(w);
  }

  // projection falls on the segment
  Node projection = v + ((w - v) * t);

  q = projection;

  return distanceTo(projection);
}


/*! \bref Compute position along the line segment
 *  Check if the node is on the line segment and return -1.0 if distance
 *  exceeds tol. Otherwise project the node onto the line segment and
 *  return is position as a percentage.
 *  \param[in] v start of segment
 *  \param[in] w end of segment
 *  \param[in] tol tolerance for distance test
 *  \return position 0.0 to 1.0 along the segment of -1.0 if it is not within tolerance
*/
double Node::positionAlongSegment(const Node &v, const Node &w, double tol) const {
  double tolSq = tol * tol;

  // i.e. |w-v|^2 ... avoid a sqrt
  double distSq = v.distanceToSquared(w);

  if (distSq == 0.0) {  // v == w case
    if (distanceToSquared(v) < tolSq)
      return 0.0;       // node == v == w case
    else
      return -1.0;      // node is not within tol
  }

  // consider the line extending the segment, parameterized as v + t (w - v)
  // we find projection of point p onto the line
  // it falls where t = [(p-v) . (w-v)] / |w-v|^2

  double t = ((*this) - v).dotProduct(w - v) / distSq;

  // beyond the v end of the segment
  if ( t < 0.0 and distanceToSquared(v) > tolSq )
    return -1.0;

  // beyond the w end of the segment
  if ( t > 1.0 and distanceToSquared(w) > tolSq )
    return -1.0;

  // projection falls on the segment
  Node projection = v + ((w - v) * t);

  if (distanceToSquared(projection) > tolSq )
    return -1.0;

  return t;
}

/*! \brief Compute the shortest distance
    * Compute the shortest distance and x,y position on the segment of the
    * closest point.
*/
double Node::distanceToSegment(double segmentX1, double segmentY1,
                               double segmentX2, double segmentY2,
                               double &qX, double &qY) const {
  Node q;

  double distance = distanceToSegment(Node(segmentX1, segmentY1),
                                      Node(segmentX2, segmentY2), q);

  qX = q.x_;
  qY = q.y_;

  return distance;
}


// Constructors

/*! \brief Construct a new Node that needs the user to set its attributes.  */
Node::Node()
  : nid_(0), id_(0), x_(0.0), y_(0.0), hint_(""), valid_(false) {
}

/*! \brief Construct a new Node and assign it \c x and \c y values.  */
Node::Node(double x, double y)
  : nid_(0), id_(0), x_(x), y_(y), hint_(""), valid_(false) {
}

/*! \brief Construct a new Node and assign it the associated values.  */
Node::Node(UID nid, UID id , double x, double y)
  : nid_(nid), id_(id), x_(x), y_(y), hint_(""), valid_(true) {
}

/*! \brief Create a new Node by parsing a string.  */
Node::Node(const std::string &line)
     : nid_(0), id_(0), x_(0.0), y_(0.0), hint_(""), valid_(true) {
  std::istringstream buffer(line);
  long int ids;
  buffer >> ids;
  buffer >> x_;
  buffer >> y_;
  hint_ = "";
  if (ids < 0) {
    valid_ = false;
  } else {
    nid_ = UID(ids);
    id_ = UID(ids);
  }
}
