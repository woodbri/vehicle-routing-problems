#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <string>



class Node {
  protected:
    int nid;
    int id;
    double x;
    double y;

  public:
    // accessors
    int getnid() const { return nid; };
    int getid() const { return id; };
    double getx() const { return x; };
    double gety() const { return y; };

    double distance(const Node &n) const ;


    // state
    bool isvalid() const { return id > -1; };
    bool isSamePos(const Node &n) const { return distance(n) == 0; };
    bool isSamePos(const Node &n, double tol) const { return distance(n) < tol; };

    // mutators
    void set(int _nid, double _x, double _y); 
    void setnid(int _nid) { nid = _nid; };
    void setid(int _id) { id = _id; };
    void setx(double _x) { x = _x; };
    void sety(double _y) {y = _y; };

    // operators
    bool operator<(const Node &n) const { return nid < n.nid; };
    bool operator==(const Node &n) const { return nid == n.nid && x == n.x && y == n.y; };
    bool operator!=(const Node &n) const { return ! (*this == n); };
    bool operator>(const Node &n) const { return nid > n.nid; };

    // vector operators
    Node operator+( const Node &v ) const;
    Node operator-( const Node &v ) const;
    Node operator*( double f ) const;
    double dotProduct( const Node &p ) const ;
    double length( const Node &p ) const ;
    double gradient( const Node &p ) const ;
    double distanceTo( const Node &p ) const ;
    double distanceToSquared( const Node &p ) const ;
    Node unit( const Node &p ) const ;
    double distanceToSegment( const Node &v, const Node &w) const ;
    double distanceToSegment( const Node &v, const Node &w, Node &q ) const;
    double distanceToSegment( double, double, double, double, double&, double&) const;

    // dump
    void dump() const;

    // constructors

    Node() ;
    Node(int _nid, double _x, double _y) ;
    Node(int _nid, int id, double _x, double _y) ;
    Node(double _x, double _y) ;
    Node(const std::string line);

    ~Node() {};

};



#endif
