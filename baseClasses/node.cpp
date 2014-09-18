
#include <iostream>
#include <sstream>

#include "node.h"


double Node::distance(const Node &n) const {
        double dx = n.x - x;
        double dy = n.y - y;
        return sqrt( dx*dx + dy*dy );
};




void Node::set(int _nid, double _x, double _y) {
        nid = _nid;
        x = _x;
        y = _y;
};

void Node::dump() const {
    std::cout << nid
              << ", " << x
              << ", " << y
              << std::endl;
};

// Vector Operations
Node  Node::operator+( const Node &v ) const { return Node( x + v.x, y + v.y ); };
Node  Node::operator-( const Node &v ) const { return Node( x - v.x, y - v.y ); };
Node  Node::operator*( double f ) const { return Node( x * f, y * f ); };
double Node::dotProduct( const Node &p ) const { return x * p.x + y * p.y; };
double Node::length( const Node &p ) const { return sqrt( x * x + y * y ); };
double Node::distanceTo( const Node &p ) const { return sqrt( distanceToSquared( p ) ); };

double Node::distanceToSquared( const Node &p ) const {
        const double dX = p.x - x;
        const double dY = p.y - y;

        return dX * dX + dY * dY;
};

Node Node::unit( const Node &p ) const {
        double scale = 0.0;
        double len = length( p );

        if (len != 0.0)
            scale = 1.0 / len;
        return p * scale;
};

double Node::distanceToSegment( const Node &v, const Node &w) const{
      Node q;
      return distanceToSegment(v,w,q);
};


double Node::distanceToSegment( const Node &v, const Node &w, Node &q ) const{

    // i.e. |w-v|^2 ... avoid a sqrt
    double distSq = v.distanceToSquared( w );

    if ( distSq == 0.0 ) { // v == w case
        q = v;
        return distanceTo( v );
    }

    // consider the line extending the segment, parameterized as v + t (w - v)
    // we find projection of point p onto the line
    // it falls where t = [(p-v) . (w-v)] / |w-v|^2

    double t = ( (*this) - v ).dotProduct( w - v ) / distSq;
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

double Node::distanceToSegment( double segmentX1, double segmentY1, double segmentX2, double segmentY2, double &qX, double &qY ) const{
    Node q;

    double distance = distanceToSegment( Node( segmentX1, segmentY1 ), Node( segmentX2, segmentY2 ), q );

    qX = q.x;
    qY = q.y;

    return distance;
}

 




// Constructors
Node::Node() {
        nid = -1;
        x = 0.0;
        y = 0.0;
};

Node:: Node(double _x, double _y) {
        nid = -1;
        x = _x;
        y = _y;
};

Node:: Node(int _nid, double _x, double _y) {
        nid = _nid;
        x = _x;
        y = _y;
};

Node::Node(std::string line) {
    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
}
