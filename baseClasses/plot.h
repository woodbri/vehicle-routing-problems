/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#ifndef PLOT_H
#define PLOT_H

#include <gd.h>
#include <limits>
#include <string>
#include <deque>
#include <cmath>

#include "twbucket.h"
#include "twpath.h"

/*! \class Plot
 * \brief A class for generating images of the locations and routes in these problems.
 * This classes plots problem data into image files. It handles the things like
 * scaling the data to fit on the image. Labeling the images and the nodes.
 *
 * The following is an example usage:
 *
 * \code
    Plot<Trashnode>  plot(datanodes);
    plot.setFile("p1.png");
    plot.setTitle("initial Solution");
    plot.drawInit();
    for (int i=0; i<fleet.size(); i++)
        plot.drawPath(fleet[i].getPath(), makeColor(i), 2, false);
    plot.drawPoints(depotids, 0xff0000, 7, true);
    plot.drawPoints(dumpids, 0x00ff00, 7, true);
    plot.drawPoints(pickupids, 0x0000ff, 3, true);
    plot.save();
 *\endcode
 *
 * \def PGROUTING
 * \brief Set this MACRO to eliminate the plotting code
 * For integration into postgresql you can compile this with -DPGROUTING to
 * eliminate the code and prevent plots from being written to the server file
 * system.
 */
template <class knode> class Plot {
private:

    const TwBucket<knode> &pts; ///< A Bucket on nodes
    std::string file;       ///< The file name to write the plot image to
    std::string title;      ///< The title to place on the image
    int width;              ///< The width of the image in pixels
    int height;             ///< The height of the image in pixels
    double extents[4];      ///< Storage for the extents of the image in user units
    double dx;              ///< The width in user units
    double dy;              ///< The height in user units
    double cx;              ///< The center x in user units
    double cy;              ///< The center y in user units
    double scale;           ///< The scale factor to apply to user values
    gdImagePtr im;          ///< A pointer to the GD image
    std::string font;       ///< The filesystem path to a ttf file

public:
    /*! \fn void calcExtents()
     * \brief Compute the extents of the nodes in the Bucket pts
     */
    void calcExtents(){//const TwBucket<knode>& pnts) {
#ifndef PGROUTING
        double extents[4];

        extents[0] = std::numeric_limits<double>::max();
        extents[1] = std::numeric_limits<double>::max();
        extents[2] = - std::numeric_limits<double>::max();
        extents[3] = - std::numeric_limits<double>::max();

        for (int i=0; i<pts.size(); i++) {
            if (pts[i].getx() < extents[0]) extents[0] = pts[i].getx();
            if (pts[i].gety() < extents[1]) extents[1] = pts[i].gety();
            if (pts[i].getx() > extents[2]) extents[2] = pts[i].getx();
            if (pts[i].gety() > extents[3]) extents[3] = pts[i].gety();
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

#endif
    }


    /*! \fn Plot(const TwBucket<knode> &_pts)
     * \brief Construct a Plot object using the supplied pts and initialize important attributes.
     */
    Plot(const TwBucket<knode> &_pts) : pts(_pts) {
#ifndef PGROUTING
        file = "plot.png";
        title = file;
        width = 800;
        height = 800;
        calcExtents();//_pts);
        im = NULL;
        // set the default to Vicky's recent ubuntu instalation font location :)
        font = "/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf";
#endif
    }

    /*! \fn void setFont(std::string _font)
     * \brief Set the path to a ttf font file
     */
    void setFont(std::string _font) { font = _font; };

    /*! \fn void setPoints(const std::deque<knode> &_pts)
     * \brief Provide a new set of points to the plot object.
     */
    void setPoints(const std::deque<knode> &_pts) {
#ifndef PGROUTING
        pts = _pts;
        calcExtents();
#endif
    };

    /*! \fn int makeColor(int i) const
     * \brief Make a valid color that can be used with the Plot object.
     */
    int makeColor(int i) const {
#ifndef PGROUTING
        int b = (i % 4 + 1) * 0x40 - 1;
        int g = ((i /  4) % 4 + 1) * 0x40 - 1;
        int r = ((i / 16) % 4 + 1) * 0x40 - 1;

        return  r*256*256 + g*256 + b;
#else
	return 0;
#endif
    };

    /*! \fn void setFile(std::string _file)
     * \brief Set the output image file and path.
     */
    void setFile(std::string _file) { file = _file; };

    /*! \fn void setTitle(std::string _title)
     * \brief Set the title to use on the image
     */
    void setTitle(std::string _title) { title = _title; };

    /*! \fn void setSize(int h, int w)
     * \brief Set the image height and width.
     */
    void setSize(int h, int w) { height = h; width = w; };

    /*! \fn int scalex(double x) const
     * \brief Scale an X value from user units into image units.
     */
    inline int scalex(double x) const {
        return (int)((x - cx) * scale + (double) width/2.0);
    }

    /*! \fn int scaley(double y) const
     * \brief Scale an Y value from user units into image units.
     */
    inline int scaley(double y) const {
        return (int) height - ((y - cy) * scale + (double) height/2.0);
    }

    /*! \fn void drawInit()
     * \brief Start the drawing process by creating an internal image structure and initializing it.
     */
    void drawInit() {
#ifndef PGROUTING
        if (im) gdImageDestroy(im);
        im = gdImageCreateTrueColor(width, height);
        gdImageFilledRectangle(im, 0, 0, width-1, height-1, 0x00ffffff);
#endif
    }

    /*! \fn void drawPath(std::deque<int> ids, int color, int thick, bool label)
     * \brief Draw a path from point to point using the ids
     * \todo Need to add labels for paths
     */
    void drawPath(std::deque<int> ids, int color, int thick, bool label) {
#ifndef PGROUTING
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot::drawInit() has not been called!\n");
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
#endif
    }

    /*! \fn void drawPoints(std::deque<int> ids, int color, int size, bool label)
     * \brief Label the nodes with their id's
     */
    void drawPoints(std::deque<int> ids, int color, int size, bool label) {
#ifndef PGROUTING
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot::drawInit() has not been called!\n");
            return;
        }

        // draw the nodes as filled circles
        for (int i=0; i<ids.size(); i++) {
            const knode &a = pts[ids[i]];
            gdImageFilledEllipse(im, scalex(a.getx()), scaley(a.gety()), size, size, color);
            // label the nodes if requested
            if (label) {
                char str[80];
                sprintf(str, "%d", a.getid());
                gdImageStringFT(im, NULL, 0x00000000, (char*) font.c_str(), 6, 0,
                                scalex(a.getx()), scaley(a.gety())-5, str);
            }
        }
#endif
    }


    /*! \fn int save()
     * \brief Save the image to the default file name.
     * return 1 on failure, 0 on success
     */
    int save() {
#ifndef PGROUTING
        // use the object variable file
        return save(file);
#else
	return 1;
#endif
    }


    /*! \fn int save()
     * \brief Save the image to the file argument.
     * \param[in] _file File and path to save the image to.
     * return 1 on failure, 0 on success
     */
    int save(std::string _file) {
#ifndef PGROUTING
        FILE *fp;

        // make sure we have been initiallized correctly
        if (!im) {
            fprintf(stderr, "Plot::drawInit() has not been called!\n");
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
#endif
    }



/* with this vehicle plot work and uses the class template */

    /*! \fn void drawPoint(const knode &a, int color, int size, bool label)
     * \brief Draw a point on the image.
     * \param[in] a The point to draw
     * \param[in] color A color defined as "0xRRGGBB" or \ref makeColor
     * \param[in] size The radius of the point in pixels
     * \param[in] label A boolean value if you want the point labeled
     */
    void drawPoint(const knode &a, int color, int size, bool label) {
#ifndef PGROUTING
        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot::drawInit() has not been called!\n");
            return;
        }

        gdImageFilledEllipse(im, scalex(a.getx()), scaley(a.gety()), size, size, color);

        if (label) {
            char str[80];
            sprintf(str, "%d", a.getid());
            gdImageStringFT(im, NULL, 0x00000000, (char *)font.c_str(), 6, 0,
                                scalex(a.getx()), scaley(a.gety())-5, str);
        }
#endif
    }

/*! \fn void drawPath( TwBucket<knode> path, int color, int thick, bool label)
 * \brief Draw a path given a Bucket of nodes.
 * \param[in] path The order Bucket of nodes to be drawn as a path.
 * \param[in] color A color defined as "0xRRGGBB" or \ref makeColor
 * \param[in] thick The thickness of the path in pixels
 * \param[in] label A boolean value if you want the path labeled.
 * \todo Currently label is ignore and the path is not labeled.
 */
    void drawPath( TwBucket<knode> path, int color, int thick, bool label) {
#ifndef PGROUTING

        // make sure drawInit() has been called
        if (!im) {
            fprintf(stderr, "Plot::drawInit() has not been called!\n");
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
#endif
    }



};

#endif
/*
// Example:
Plot<Trashnode>  plot(datanodes);
plot.setFile("p1.png");
plot.setTitle("initial Solution");
plot.drawInit();
for (int i=0; i<fleet.size(); i++)
    plot.drawPath(fleet[i].getPath(), makeColor(i), 2, false);
plot.drawPoints(depotids, red, 7, true);
plot.drawPoints(dumpids, green, 7, true);
plot.drawPoints(pickupids, blue, 3, true);
plot.save(); 
*/
