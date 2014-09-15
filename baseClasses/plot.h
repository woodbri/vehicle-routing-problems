#ifndef PLOT1_H
#define PLOT1_H

#include <gd.h>
#include <limits>
#include <string>
#include <deque>
#include <cmath>
#include "twpath.h"

template <class knode> class Plot {
private:

    const Twpath<knode> &pts;
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
    std::string font;

public:
    void calcExtents(const Twpath<knode>& pnts) {
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


    Plot(const Twpath<knode> &_pts) : pts(_pts) {
        file = "plot1.png";
        title = file;
        width = 800;
        height = 800;
        calcExtents(_pts);
        im = NULL;
        // set the default to Vicky's font location :)
        font = "/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf";
    }

    void setFont(std::string _font) { font = _font; };

    void setPoints(const std::deque<knode> &_pts) {
        pts = _pts;
        calcExtents();
    };

    int makeColor(int i) const {
        int b = (i % 4 + 1) * 0x40 - 1;
        int g = ((i /  4) % 4 + 1) * 0x40 - 1;
        int r = ((i / 16) % 4 + 1) * 0x40 - 1;

        return  r*256*256 + g*256 + b;
    };

    void setFile(std::string _file) { file = _file; };

    void setTitle(std::string _title) { title = _title; };

    void setSize(int h, int w) { height = h; width = w; };

    inline int scalex(double x) const {
        return (int)((x - cx) * scale + (double) width/2.0);
    }


    inline int scaley(double y) const {
        return (int) height - ((y - cy) * scale + (double) height/2.0);
    }


    void drawInit() {
        if (im) gdImageDestroy(im);
        im = gdImageCreateTrueColor(width, height);
        gdImageFilledRectangle(im, 0, 0, width-1, height-1, 0x00ffffff);
    }


/*
    void drawPath(const Twpath<knode>& path, int color, int thick, bool label) {
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot1::drawInit() has not been called!\n");
            return;
        }

        // set the line thickness for drawing
        gdImageSetThickness(im, thick);

        // extract the color into RGB values and set the line draw color
        int blue = color % 256;
        int green = (color / 256) % 256;
        int red = (color / 65536) % 256;
        gdImageSetAntiAliased(im, gdImageColorExactAlpha(im, red, green, blue, 0));

        // draw the path based on a list of node ids
        for (int i=0; i<path.size()-1; i++) {
            const knode &a = path[i];
            const knode &b = path[i+1];
            gdImageLine(im, scalex(a.getx()), scaley(a.gety()),
                            scalex(b.getx()), scaley(b.gety()), gdAntiAliased);
        }

        // label the paths if requested
        if (label) {
            // TODO pick midpoint of 2nd segment calc angle of segment
            //  and label along it
        }
    }
*/

    void drawPath(std::deque<int> ids, int color, int thick, bool label) {
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot1::drawInit() has not been called!\n");
            return;
        }

        // set the line thickness for drawing
        gdImageSetThickness(im, thick);

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


    void drawPoints(std::deque<int> ids, int color, int size, bool label) {
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot1::drawInit() has not been called!\n");
            return;
        }

        // draw the nodes as filled circles
        for (int i=0; i<ids.size(); i++) {
            const knode &a = pts[ids[i]];
            gdImageFilledEllipse(im, scalex(a.getx()), scaley(a.gety()), size, size, color);
            // label the nodes if requested
            if (label) {
                char str[80];
                sprintf(str, "%d", a.getnid());
                gdImageStringFT(im, NULL, 0x00000000, (char*) font.c_str(), 6, 0,
                                scalex(a.getx()), scaley(a.gety())-5, str);
            }
        }
    }


/*
    void drawPoints(std::deque<knode> pnts, int color, int size, bool label) {
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot1::drawInit() has not been called!\n");
            return;
        }

        // draw the nodes as filled circles
        for (int i=0; i<pnts.size(); i++) {
            const knode &a = pnts[i];
            gdImageFilledEllipse(im, scalex(a.getx()), scaley(a.gety()), size, size, color);
            // label the nodes if requested
            if (label) {
                char str[80];
                sprintf(str, "%d", a.getnid());
                gdImageStringFT(im, NULL, 0x00000000, (char *)font.c_str(), 6, 0,
                                scalex(a.getx()), scaley(a.gety())-5, str);
            }
        }
    }
*/

    int save() {
        // use the object variable file
        return save(file);
    }


    int save(std::string _file) {
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
        gdImageStringFT(im, NULL, 0x00000000, (char*) font.c_str(), 8, 0, 5, 20, (char *)(title.c_str()));

        // save the image and clean up
        gdImagePng(im, fp);
        fclose(fp);
        gdImageDestroy(im);
        im = NULL;
        return 0;
    }



/* with this vehicle plot work and uses the class template */

    void drawPoint(const knode &a, int color, int size, bool label) {
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot1::drawInit() has not been called!\n");
            return;
        }

        gdImageFilledEllipse(im, scalex(a.getx()), scaley(a.gety()), size, size, color);

        if (label) {
            char str[80];
            sprintf(str, "%d", a.getnid());
            gdImageStringFT(im, NULL, 0x00000000, (char *)font.c_str(), 6, 0,
                                scalex(a.getx()), scaley(a.gety())-5, str);
        }
    }


    void drawPath( Twpath<knode> path, int color, int thick, bool label) {
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot1::drawInit() has not been called!\n");
            return;
        }

        // set the line thickness for drawing
        gdImageSetThickness(im, thick);

        // extract the color into RGB values and set the line draw color
        int blue = color % 256;
        int green = (color / 256) % 256;
        int red = (color / 65536) % 256;
        gdImageSetAntiAliased(im, gdImageColorExactAlpha(im, red, green, blue, 0));

        // draw the path based on a list of node ids
        for (int i=0; i<path.size()-1; i++) {
            const knode &a = path[i];
            const knode &b = path[i+1];
            gdImageLine(im, scalex(a.getx()), scaley(a.gety()),
                            scalex(b.getx()), scaley(b.gety()), gdAntiAliased);
        }

        if (label) {
            // TODO pick midpoint of 2nd segment calc angle of segment
            //  and label along it
        }
    }



};

#endif
/*
// Example:
Plot1<Trashnode>  plot(datanodes);
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
