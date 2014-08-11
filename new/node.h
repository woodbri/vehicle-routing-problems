#ifndef NODE_H
#define NODE_H

#include <cmath>
#include <string>

class Node {
  private:
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
