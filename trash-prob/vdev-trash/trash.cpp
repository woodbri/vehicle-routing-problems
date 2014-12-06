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

#include "logger.h"

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif



//#include "Library/OSRM.h"

#include "trashconfig.h"
#include "feasableSol.h"
#include "tabuopt.h"


void Usage() {
    std::cout << "Usage: trash file (no extension)\n";
}

#ifdef DOPLOT
static std::string font = "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf";
#endif

/* Logging Severity Levels
    0   INFO
    1   WARNING
    2   ERROR
    3   FATAL  NOTE FATAL is converted to ERROR when NDEBUG is defined

    DLOG(<level>) << "message";
    DLOG_IF(<level>, <cond>) << "message";
    DLOG_EVERY_N(<level>, <n>) << "message";
    CHECK(<cond>) << "message";  // print "message" and FATAL if false

    http://google-glog.googlecode.com/svn/trunk/doc/glog.html

    GLOG_logtostderr=1 ./bin/trash ...  // to get output to terminal
*/

int main(int argc, char **argv) {

    FLAGS_log_dir = "./logs/";
    google::InitGoogleLogging("vdev/Trash");
    //FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::ERROR;
    FLAGS_minloglevel = google::INFO;

    if (argc < 2) {
        Usage();
        return 1;
    }

    std::string infile = argv[1];

    #ifdef WITHOSRM
    // MUST call this once to initial communications via cURL
    cURLpp::Cleanup myCleanup;
    #endif

    try {
	#ifdef LOG
	#ifdef OSRMCLIENT
	osrm->useOsrm(true);
	osrm->testOsrmClient();
	osrm->useOsrm(false);
	#endif
	#endif

	#ifdef DOSTATS
        Timer starttime;
	#endif

        #ifdef DOPLOT 
        CONFIG->set("plotDir", "./logs/");
	#endif
        CONFIG->dump("CONFIG");

       
        FeasableSol tp(infile);
	

	#ifndef LOG
        tp.dump();
        DLOG(INFO) << "FeasableSol time: " << starttime.duration();
	#endif
	#ifdef DOSTATS
        STATS->set("zzFeasableSol time", starttime.duration());
	#endif

        tp.setInitialValues();
        tp.computeCosts();
	#ifdef DOSTATS
        STATS->set("zInitial cost", tp.getCost());
        STATS->set("yNode count", tp.getNodeCount());
        STATS->set("yVehicle count", tp.getFleetSize());

        Timer searchtime;
	#endif

        TabuOpt ts(tp);
        ts.setMaxIteration(1000);
        ts.search();

	#ifdef DOSTATS
        STATS->set("zzSearch time", searchtime.duration());
	#endif

        Solution best = ts.getBestSolution();
        best.computeCosts();

	#ifdef DOSTATS
        STATS->set("zzTotal time", starttime.duration());
	#endif

	#ifndef LOG
        best.dump();
	#endif

	#ifdef DOSTATS
        STATS->set("zBest cost", best.getCost());
        STATS->set("zBest distance", best.getDistance());

        STATS->dump("Final");
	#endif

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




