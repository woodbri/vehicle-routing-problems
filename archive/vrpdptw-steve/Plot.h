#ifndef PLOT_H
#define PLOT_H

#include <string>

class Route;    // forward reference

#include "Problem.h"
#include "Solution.h"

class Plot {
  public:
    Problem &P;
    Solution &S;

    double dx;
    double dy;
    double cx;
    double cy;
    double scale;
    int width;
    int height;

    Plot(Solution &s) : P(s.P), S(s) {};

    int scalex(double x);
    int scaley(double y);
    int out(std::string file, bool labelNodes, int iwidth, int iheight, char* title);

};

#endif
