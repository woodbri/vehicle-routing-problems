#ifndef PLOT_H
#define PLOT_H

#include <limits>

#include <string>
#include <vector>
#include <math.h>

//class Route;    // forward reference

//#include "problem.h"
//#include "tau.h"

class Plot {
  private:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<int> pointclr;
    std::vector<int> lineclr;
    std::vector<int> label;
    int size;

    double dx;
    double dy;
    double cx;
    double cy;
    double scale;

    std::string file;
    std::string title;
    int width;
    int height;
    double extents[4];
public:
    Plot();

    Plot(std::vector<double> iX, std::vector<double> iY, std::vector<int> pc, std::vector<int> lc,std::vector<int> ilabel);
    void setextents();
    void setXY(std::vector<double> iX, std::vector<double> iY);
    void setLabel(std::vector<int> ilabel){ label=label; };
    void setPointColor(std::vector<int> pc){ pointclr=pc; };
    void setLineColor(std::vector<int> lc){ lineclr=lc; };
    void setFile(std::string ifile) { file=ifile; };
    void setTitle(std::string ititle) { title=ititle; };
    void setsize(int iwidth, int iheight){ width=iwidth; height=iheight; };
    int scalex(double x);
    int scaley(double y);
    //int out(std::string file, bool labelNodes, int iwidth, int iheight, char* title);
    int plot(bool labelNodes);

};

#endif
