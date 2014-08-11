#ifndef PATHNODE_H
#define PATHNODE_H

//#include <stdexcept>
//#include <algorithm>
#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <math.h>

#include "node.h"

class pathNode {
  private:
    Node *node;
    const double cargoLimit;

  public:
    bool twv;
    bool cv;
    int twvTot;
    int cvTot;
    double cargo;
    double distPrev;
    double totDistFromDepot;

    Node& getnode(){return *node;};
    bool isdepot() const {return node->isvehicle();}
    bool ispickup() const {return node->ispickup();}
    bool isdump() const {return  node->isdump();}
    int getnid() const {return node->getnid();}
    double getx() const {return node->getx();}
    double gety() const {return node->gety();}

    


    void dump() {
        node->dump();
        std::cout<<" twv="<<twv
                 <<", cv="<<cv
                 <<", twvTot="<<twvTot
                 <<", cvTot="<<cvTot
                 <<", cargo="<<cargo
                 <<", distPrev="<<distPrev
                 <<", totDistFromDepot="<<totDistFromDepot
                 <<"\n";
    }


    void copyvalues (const pathNode &other) {
        node = other.node;
        twv = other.twv;
        cv = other.cv;
        twvTot = other.twvTot;
        cvTot = other.cvTot;
        cargo = other.cargo;
        distPrev = other.distPrev;
        totDistFromDepot = other.totDistFromDepot;
    };


    pathNode(Node &n) {
        node = &n;
        twv = false;
        cv = false;
        twvTot = 0;
        cvTot = 0;
        cargo = 0;
        distPrev = 0;
        totDistFromDepot = 0;
    };


    pathNode(const pathNode &other) : node(other.node) {
        copyvalues(other);
    };

    pathNode& operator=(const pathNode &other) {
        copyvalues(other);
    };        

    double setDistPrev(const Node &prev) {
        distPrev=node->distance(prev);
    };

    double getDistFromStop(pathNode &other) {
        return node->distance(other.getnode());
    };

    void   setValues(pathNode &prev);
    void   setValues(const Node &depot);

    ~pathNode(){};
};    

#endif
