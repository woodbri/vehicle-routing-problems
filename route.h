#ifndef ROUTE_H
#define ROUTE_H

#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "path.h"
#include "problem.h"

class Route {

  private:


  public:
    int rid;

    Problem& P;

    Path routePath;

    double D;      // duration
    int TWV;    // TW violations
    int CV;     // capacity violations
    double cost;

    Route(Problem& p);

    Route &operator = (const Route &r) { P = r.P; return *this; };

    int getnid(int i) const { return routePath.getnid(i); }
    double getx(const int i) const {routePath.getx(i);}
    double gety(const int i) const {routePath.gety(i);}

};

#endif

