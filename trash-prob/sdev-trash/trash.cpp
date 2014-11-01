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

#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#include <stdio.h>

#include "timer.h"
#include "trashconfig.h"
#include "trashstats.h"
#include "osrm.h"
#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "feasableSolLoop.h"
#include "tabuopt.h"



void Usage() {
    std::cout << "Usage: trash file (no extension)\n";
}

static std::string font = "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf";

int main(int argc, char **argv) {

    if (argc < 2) {
        Usage();
        return 1;
    }

    std::string infile = argv[1];

    // MUST call this once to initial communications via cURL
    cURLpp::Cleanup myCleanup;

    try {
        Timer starttime;

        CONFIG->set("plotDir", "./logs/");
        //CONFIG->set("osrmBaseUrl", "http://imaptools.com:5000/");
        CONFIG->dump("CONFIG");

       
        FeasableSolLoop tp(infile);
        //tp.dump();

        std::cout << "FeasableSol time: " << starttime.duration() << std::endl;
        STATS->set("zzFeasableSol time", starttime.duration());

        tp.setInitialValues();
        tp.computeCosts();
        STATS->set("zInitial cost", tp.getCost());
        STATS->set("yNode count", tp.getNodeCount());
        STATS->set("yVehicle count", tp.getFleetSize());

        Timer searchtime;

        TabuOpt ts(tp);
        ts.setMaxIteration(1000);
        ts.search();

        STATS->set("zzSearch time", searchtime.duration());

        Solution best = ts.getBestSolution();
        best.computeCosts();

        STATS->set("zzTotal time", starttime.duration());

        best.dump();

        STATS->set("zBest cost", best.getCost());
        STATS->set("zBest distance", best.getDistance());

        STATS->dump("Final");

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




