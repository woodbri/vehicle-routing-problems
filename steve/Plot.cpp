#include <gd.h>
#include <stdio.h>

#include "Plot.h"

inline int Plot::scalex(double x) {
    return (int)((x - cx) * scale + (double) width/2.0);
}

inline int Plot::scaley(double y) {
    return (int) height - ((y - cy) * scale + (double) height/2.0);
}

int Plot::out(std::string file, bool labelNodes, int iwidth, int iheight, char *title) {
    FILE *fp;
    gdImagePtr im;

    width = iwidth;
    height = iheight;
    dx =  P.extents[2] - P.extents[0];
    dy =  P.extents[3] - P.extents[1];
    cx = (P.extents[2] + P.extents[0]) / 2.0;
    cy = (P.extents[3] + P.extents[1]) / 2.0;
    scale = fmin((double)width/dx, (double)height/dy);

    im = gdImageCreateTrueColor(iwidth, iheight);
    gdImageFilledRectangle(im, 0, 0, iwidth-1, iheight-1, 0x00ffffff);

    fp = fopen(file.c_str(), "wb");
    if (!fp) {
        fprintf(stderr, "Can't save plot as png image.\n");
        gdImageDestroy(im);
        return 1;
    }

    gdImageSetThickness(im, 1);
    gdImageSetAntiAliased(im, gdImageColorExactAlpha(im, 0, 0, 0, 0));

    // draw the lines for the routes
    for (int i=0; i<S.R.size(); i++) {
        if (! S.R[i].path.size()) continue;
        int x1, y1, x2, y2;
        x1 = scalex(P.N[0].x);
        y1 = scaley(P.N[0].y);
        x2 = scalex(P.N[S.R[i].path[0]].x);
        y2 = scaley(P.N[S.R[i].path[0]].y);
        gdImageLine(im, x1, y1, x2, y2, gdAntiAliased);
        x1 = scalex(P.N[0].x);
        y1 = scaley(P.N[0].y);
        x2 = scalex(P.N[S.R[i].path.back()].x);
        y2 = scaley(P.N[S.R[i].path.back()].y);
        gdImageLine(im, x1, y1, x2, y2, gdAntiAliased);
        for (int j=1; j<S.R[i].path.size(); j++) {
            x1 = scalex(P.N[S.R[i].path[j-1]].x);
            y1 = scaley(P.N[S.R[i].path[j-1]].y);
            x2 = scalex(P.N[S.R[i].path[j]].x);
            y2 = scaley(P.N[S.R[i].path[j]].y);
            gdImageLine(im, x1, y1, x2, y2, gdAntiAliased);
        }
    }

    // draw the nodes RED for depot, Grreen for pickup Blue for delivery
    int color;
    for (int i=0; i<P.N.size(); i++) {
        double x = scalex(P.N[i].x);
        double y = scaley(P.N[i].y);

        if (i == 0) {   // depot
            gdImageFilledEllipse(im, x, y, 11, 11, 0x00ff0000);
        }
        else if (P.N[i].demand >0) {  // pickup
            gdImageFilledEllipse(im, x, y, 7, 7, 0x0000ff00);
        }
        else {  //delivery
            gdImageFilledEllipse(im, x, y, 7, 7, 0x000000ff);
        }
    }

    char *font = (char *)"/u/data/maps/fonts/verdana.ttf";

    // label the nodes if requested
    if (labelNodes) {
        for (int i=0; i<P.N.size(); i++) {
            char str[80];
            int x, y;
            x = scalex(P.N[i].x);
            y = scaley(P.N[i].y);
            sprintf(str, "%d", i);
            gdImageStringFT(im, NULL, 0x00000000, font, 6, 0, x, y-5, str);
        }
    }

    gdImageStringFT(im, NULL, 0x00000000, font, 8, 0, 5, 20, title);

    gdImagePng(im, fp);
    fclose(fp);
    gdImageDestroy(im);
    return 0;
}

