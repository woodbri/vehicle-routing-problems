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
    Plot() { 
       file="graph.png";
       width=800;
       height=800;
       size=0;
       extents[0] = std::numeric_limits<double>::max();
       extents[1] = std::numeric_limits<double>::max();
       extents[2] = std::numeric_limits<double>::min();
       extents[3] = std::numeric_limits<double>::min();
    };

    Plot(std::vector<double> iX, std::vector<double> iY, std::vector<int> pc, std::vector<int> lc,std::vector<int> ilabel){
       file="graph.png";
       width=800;
       height=800;
       size=iX.size();
       x=iX;
       y=iY;
       pointclr=pc;
       lineclr=lc;
       label=ilabel;
       setextents();
    };


    void setextents(){
       extents[0] = std::numeric_limits<double>::max();
       extents[1] = std::numeric_limits<double>::max();
       extents[2] = std::numeric_limits<double>::min();
       extents[3] = std::numeric_limits<double>::min();
       for (int i=0;i<size;i++){
           if (x[i] < extents[0]) extents[0] = x[i];
           if (y[i] < extents[1]) extents[1] = y[i];
           if (x[i] > extents[2]) extents[2] = x[i];
           if (y[i] > extents[3]) extents[3] = y[i];
       };
       extents[0] -= (extents[2] - extents[0]) * 0.02;
       extents[2] += (extents[2] - extents[0]) * 0.02;
       extents[1] -= (extents[3] - extents[1]) * 0.02;
       extents[3] += (extents[3] - extents[1]) * 0.02;
       dx =  extents[2] - extents[0];
       dy =  extents[3] - extents[1];
       cx = (extents[2] + extents[0]) / 2.0;
       cy = (extents[3] + extents[1]) / 2.0;
       scale = fmin((double)width/dx, (double)height/dy);
    };


    void setXY(std::vector<double> iX, std::vector<double> iY){
        size=x.size();
        x=iX;
        y=iY;
    };
    
    void setLabel(std::vector<int> ilabel){
           label=label;
    };

    void setPointColor(std::vector<int> pc){
           pointclr=pc;
    };

    void setLineColor(std::vector<int> lc){
           lineclr=lc;
    };

    void setFile(std::string ifile) {
        file=ifile;
    };
  
    void setTitle(std::string ititle) {
        title=ititle;
    };

    void setsize(int iwidth, int iheight){
       width=iwidth; height=iheight;
    };
     
    
    int scalex(double x);
    int scaley(double y);
    int out(std::string file, bool labelNodes, int iwidth, int iheight, char* title);
    int plot(bool labelNodes);

};

#endif
