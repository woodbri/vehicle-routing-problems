#ifndef NODE_H
#define NODE_H

#include <iostream>

class Node {
protected:
    int nid;        // node number (0 = depot
    double x;       // x location
    double y;       // y location

public:
/*Accesors */
    int getnid() const {return nid;};
    int getx() const {return x;};
    int gety() const {return y;};
/*Mutators***/
    int setnid(int _nid)  { nid=_nid;};
    int setx(double _x)   { x=_x;};
    int sety(double _y)   { y=_y;};
/****/
    bool isvalid() const { return nid > -1; };
    bool isSamePos(const Node &n) { return distance(n) == 0; };
    bool isSamePos(const Node &n, double tol) { return distance(n) < tol; };
    int quadrant() const;
    int quadrant( const Node &from) const;
    double distance(const Node &n2) const ;
    virtual void dump() const;
    virtual void debugdump() const;
/* operators */
    bool operator<(const Node &n) const { return nid < n.nid; };
    bool operator==(const Node &n) const { return nid == n.nid && x == n.x && y == n.y; };
    bool operator!=(const Node &n) const { return ! (*this == n); };
    bool operator>(const Node &n) const { return nid > n.nid; };

/* constructors & destructors */
    Node (const int &_nid,const double &_x, const double &_y) {
      nid=_nid;x=_x;y=_y;};
    Node(const Node &node) {
      nid=node.nid;x=node.x;y=node.y;};
    Node() {
      nid = -1;
      x = 0.0;
      y = 0.0;
    };
    Node(std::string line);
    ~Node() {};

};

#endif
