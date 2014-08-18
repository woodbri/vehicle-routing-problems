#include <gd.h>
#include <limits>

#include "plot1.h"


inline int Plot1::scalex(double x) const {
    return (int)((x - cx) * scale + (double) width/2.0);
}


inline int Plot1::scaley(double y) const {
    return (int) height - ((y - cy) * scale + (double) height/2.0);
}


Plot1::Plot1(const std::vector<knode> &_pts) : pts(_pts) {
    file = "plot1.png";
    title = file;
    width = 800;
    height = 800;
    calcExtents();
    im = NULL;
}



void Plot1::setPoints(const std::vector<knode> &_pts) {
    pts = _pts;
    calcExtents();
};


// private method
void Plot1::calcExtents() {
    double extents[4];

    extents[0] = std::numeric_limits<double>::max();
    extents[1] = std::numeric_limits<double>::max();
    extents[2] = std::numeric_limits<double>::min();
    extents[3] = std::numeric_limits<double>::min();

    for (int i=0; i<pts.size(); i++) {
        if (pnts[i].getx() < extents[0]) extents[0] = pnts[i].getx();
        if (pnts[i].gety() < extents[1]) extents[1] = pnts[i].gety();
        if (pnts[i].getx() > extents[2]) extents[2] = pnts[i].getx();
        if (pnts[i].gety() > extents[3]) extents[3] = pnts[i].gety();
    }

    extents[0] -= (extents[2] - extents[0]) * 0.02;
    extents[2] += (extents[2] - extents[0]) * 0.02;
    extents[1] -= (extents[3] - extents[1]) * 0.02;
    extents[3] += (extents[3] - extents[1]) * 0.02;

    dx =  extents[2] - extents[0];
    dy =  extents[3] - extents[1];
    cx = (extents[2] + extents[0]) / 2.0;
    cy = (extents[3] + extents[1]) / 2.0;

    scale = fmin((double)width/dx, (double)height/dy);
}


void Plot1::drawInit() {
    if (im) gdImageDestroy(im);
    im = gdImageCreateTrueColor(width, height);
    gdImageFilledRectangle(im, 0, 0, width-1, height-1, 0x00ffffff);
}


 template <class knode>
void Plot1<knode>::drawPath(std::vector<int> ids, int color, bool label) {
    // make sure drawInit() has been called
    if (!im) {
        fprintf(stderr, "Plot1::drawInit() has not been called!\n");
        return 1;
    }

    // set the line thickness for drawing
    gdImageSetThickness(im, 1);

    // extract the color into RGB values and set the line draw color
    int blue = color % 256;
    int green = (color / 256) % 256;
    int red = (color / 65536) % 256;
    gdImageSetAntiAliased(im, gdImageColorExactAlpha(im, red, green, blue, 0));

    // draw the path based on a list of node ids
    for (int i=0; i<ids.size()-1; i++) {
        const knode &a = pts[ids[i]];
        const knode &b = pts[ids[i+1]];
        gdImageLine(im, scalex(a.getx()), scaley(a.gety()),
                        scalex(b.getx()), scaley(b.gety()), gdAntiAliased);
    }

    // label the paths if requested
    if (label) {
        // TODO pick midpoint of 2nd segment calc angle of segment
        //  and label along it
    }
}


void Plot1::drawPoints(std::vector<int> ids, int color, bool label) {
    // make sure drawInit() has been called
    if (!im) {
        fprintf(stderr, "Plot1::drawInit() has not been called!\n");
        return 1;
    }

    // draw the nodes as filled circles
    for (int i=0; i<ids.size()-1; i++) {
        const knode &a = pts[ids[i]];
        gdImageFilledEllipse(im, scalex(a.getx()), scaley(a.gety()), 7, 7, color);
    }

    // label the nodes if requested
    if (label) {
        char str[80];
        for (int i=0; i<ids.size(); i++) {
            const knode &a = pts[ids[i]];
            sprintf(str, "%d", label[i]);
            gdImageStringFT(im, NULL, 0x00000000, font, 6, 0,
                            scalex(a.getx()), scaley(a.gety())-5, str);
        }
    }
}


int Plot1::save() {
    // use the object variable file
    return save(file);
}


int Plot1::save(std::string _file) {
    FILE *fp;

    // make sure we have been initiallized correctly
    if (!im) {
        fprintf(stderr, "Plot1::drawInit() has not been called!\n");
        return 1;
    }

    // oprn the file to write the image to
    fp = fopen(_file.c_str(), "wb");
    if (!fp) {
        fprintf(stderr, "Can't save plot as png image.\n");
        gdImageDestroy(im);
        return 1;
    }

    // draw the title
    char *font = (char *)"/u/data/maps/fonts/verdana.ttf";
    gdImageStringFT(im, NULL, 0x00000000, font, 8, 0, 5, 20, title.c_str());

    // save the image and clean up
    gdImagePng(im, fp);
    fclose(fp);
    gdImageDestroy(im);
    im = NULL;
    return 0;
}

