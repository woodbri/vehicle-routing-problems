#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <string>

class Node {
  protected:
    int nid;
    double x;
    double y;

  public:
    int getnid() const { return nid; };
    int getx() const { return x; };
    int gety() const { return y; };

    double distance(const Node &n) const {
        double dx = n.x - x;
        double dy = n.y - y;
        return sqrt( dx*dx + dy*dy );
    };

    void dump() const;

    bool isvalid() const { return nid > -1; };
    bool isSamePos(const Node &n) { return distance(n) == 0; };
    bool isSamePos(const Node &n, double tol) { return distance(n) < tol; };

    void setvalues(int _nid, double _x, double _y) {
        nid = _nid;
        x = _x;
        y = _y;
    };

    void setnid(int _nid) { nid = _nid; };
    void setx(double _x) { x = _x; };
    void sety(double _y) {y = _y; };

    bool operator<(const Node &n) const { return nid < n.nid; };
    bool operator==(const Node &n) const { return nid == n.nid && x == n.x && y == n.y; };
    bool operator!=(const Node &n) const { return ! (*this == n); };
    bool operator>(const Node &n) const { return nid > n.nid; };


    Node() {
        nid = -1;
        x = 0.0;
        y = 0.0;
    };

    Node(int _nid, double _x, double _y) {
        nid = _nid;
        x = _x;
        y = _y;
    };

    Node(const std::string line);

    ~Node() {};

};

#endif
