#include <gd.h>
#include <stdio.h>

#include "plot.h"

inline int Plot::scalex(double x) {
    return (int)((x - cx) * scale + (double) width/2.0);
}

inline int Plot::scaley(double y) {
    return (int) height - ((y - cy) * scale + (double) height/2.0);
}


Plot::Plot() {
       file="graph.png";
       width=800;
       height=800;
       size=0;
       extents[0] = std::numeric_limits<double>::max();
       extents[1] = std::numeric_limits<double>::max();
       extents[2] = std::numeric_limits<double>::min();
       extents[3] = std::numeric_limits<double>::min();
}


Plot::Plot(std::vector<double> iX, std::vector<double> iY, std::vector<int> pc, std::vector<int> lc,std::vector<int> ilabel){
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


void Plot::setextents(){
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

void Plot::setXY(std::vector<double> iX, std::vector<double> iY){
        size=x.size();
        x=iX;
        y=iY;
};



//int Plot::out(std::string file, bool labelNodes, int iwidth, int iheight, char *title) {};

int Plot::plot(bool labelNodes) {
    FILE *fp;
    gdImagePtr im;
    int color;

    im = gdImageCreateTrueColor(width, height);

    gdImageFilledRectangle(im, 0, 0, width-1, height-1, 0x00ffffff);

    fp = fopen(file.c_str(), "wb");
    if (!fp) {
        fprintf(stderr, "Can't save plot as png image.\n");
        gdImageDestroy(im);
        return 1;
    }

    gdImageSetThickness(im, 1);
    //gdImageSetAntiAliased(im, gdImageColorExactAlpha(im, 0, 0, 0, 0));

    // draw the lines for the routes
    for (int i=0; i<size-1; i++) {
        gdImageSetAntiAliased(im, gdImageColorExactAlpha(im, lineclr[i], lineclr[i]*5, lineclr[i]*10, 0));
        gdImageLine(im, scalex(x[i]), scaley(y[i]),scalex(x[i+1]), scaley(y[i+1]), gdAntiAliased);
        //gdImageLine(im, scalex(x[i]), scaley(y[i]),scalex(x[i]), scaley(y[i]), lineclr[i]);
    }

    // draw the nodes RED for depot, Grreen for pickup Blue for delivery
    for (int i=0; i<pointclr.size(); i++) {
        gdImageFilledEllipse(im, scalex(x[i]), scaley(y[i]), 7, 7, pointclr[i]);
    }

    char *font = (char *)"/u/data/maps/fonts/verdana.ttf";

    // label the nodes if requested
    if (labelNodes) {
        char str[80];
        for (int i=0; i<label.size(); i++) {
            sprintf(str, "%d", label[i]);
            gdImageStringFT(im, NULL, 0x00000000, font, 6, 0, scalex(x[i]), scaley(y[i])-5, str);
        }
    }
    char tit[80];
    sprintf(tit, "%s", title.c_str());
    gdImageStringFT(im, NULL, 0x00000000, font, 8, 0, 5, 20, tit);

    gdImagePng(im, fp);
    fclose(fp);
    gdImageDestroy(im);
    return 0;
}

