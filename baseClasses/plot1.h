#ifndef PLOT1_H
#define PLOT1_H

#include "node.h"

template <class knode> class Plot1 {
private:
    const std::vector<knode> &pts;
    std::string file;
    std::string title;
    int width;
    int height;
    double extents[4];
    double dx;
    double dy;
    double cx;
    double cy;
    double scale;
    gdImagePtr im;

    // private method no need to call this externally
    void calcExtents();

public:
    Plot1(std::vector<knode> &_pts);
    void setPoints(const std::vector<knode> &_pts);
    void setFile(std::string _file) { file = _file; }:
    void setTitle(std::string _title) { title = _title; };
    void setSize(int h, int w) { height = h; width = w; };

    int scalex(double x) const;
    int scaley(double y) const;

    void drawInit();
    void drawPath(std::vector<int> ids, int color, bool label);
    void drawPoints(std::vector<int> ids, int color, bool label);
    void save();
    void save(std::string _file);
};
#endif
/*
// Example:
Plot plot(datanodes);
plot.setFile("p1.png");
plot.setTitle("initial Solution");
plot.drawInit();
for (int i=0; i<fleet.size(); i++)
    plot.drawPath(fleet[i].getPath(), somecolors[i], false);
plot.drawPoints(depotids, red, true);
plot.drawPoints(dumpids, green, true);
plot.drawPoints(pickupids, blue, true);
plot.save(); 
*/
