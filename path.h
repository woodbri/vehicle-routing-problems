#ifndef PATH_H
#define PATH_H

#include <deque>

#include "node.h"
#include "pathnode.h"


class Path {
  private:
    Node *depot;
    int rid;
    double D;      // duration
    int TWV;    // TW violations
    int CV;     // capacity violations

    std::deque<pathNode> path;      // node ids along the path and more

  public:

    Path (Node &d){
        depot = &d;
        //twv_depot = false;
        //cv_depot = false;
        D = TWV = CV = 0;
        pathNode dep(d);
        path.push_back(dep);
    };

    ~Path() {};

    int size() {return path.size();};
    double getx(const int i) const { return path[i].getx(); };
    double gety(const int i) const { return path[i].gety(); };
    int getnid(int i) const { return path[i].getnid(); };
    bool feasable() { return TWV == 0 and CV == 0;}
    bool hascv()const { return CV != 0;}
    bool hastwv()const { return TWV != 0;}

};

#endif

