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
#include "timer.h"
#include "trashconfig.h"
#include "stats.h"
#include "node.h"
#include "osrmclient.h"
//#include "vrposrm.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "feasableSolLoop.h"
//#include "tabusearch.h"
#include "tabuopt.h"

void Usage() {
    std::cout << "Usage: trash file (no extension)\n";
}

static std::string font = "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf";

int testOsrmClient() {
    Timer t0;
    OsrmClient* oc = OsrmClient::Instance();
    oc->dump();
    if (oc->getStatus() == -1) {
        DLOG(INFO) << oc->getErrorMsg() << std::endl;
        return -1;
    }
    oc->setWantGeometry( true );
    oc->addViaPoint(-34.88124, -56.19048);
    oc->addViaPoint(-34.89743, -56.12447);
    oc->dump();
    if (oc->getOsrmViaroute()) {
        DLOG(INFO) << "getOsrmViaroute: Failed!\n";
        DLOG(INFO) << oc->getErrorMsg() << std::endl;
        return -1;
    }
    oc->dump();
    double time;
    if (oc->getOsrmTime( time )) {
        DLOG(INFO) << "getOsrmTime Failed!\n";
        DLOG(INFO) << oc->getErrorMsg() << std::endl;
        return -1;
    }
    DLOG(INFO) << "getOsrmTime: " << time << std::endl;
    DLOG(INFO) << "duration: " << t0.duration() << std::endl;

    std::deque<Node> geom;
    if (oc->getOsrmGeometry( geom )) {
        DLOG(INFO) << "getOsrmGeometry Failed!\n";
        DLOG(INFO) << oc->getErrorMsg() << std::endl;
        return -1;
    }
    for (int i=0; i<geom.size(); ++i) {
        DLOG(INFO) << "geom[" << i << "]: ";
        geom[i].dump();
    }
    return 0;
}


/* Logging Severity Levels
    0   INFO
    1   WARNING
    2   ERROR
    3   FATAL  NOTE FATAL is converted to ERROR when NDEBUG is defined

    DLOG(<level>) << "message";
    DLOG_IF(<level>, <cond>) << "message";
    DLOG_EVERY_N(<level>, <n>) << "message";
    CHECK(<cond>) << "message";  // print "message" and FATAL if false
*/


int main(int argc, char **argv) {

    FLAGS_log_dir = "./logs/";
    google::InitGoogleLogging("sdev/Trash");
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::ERROR;
    FLAGS_minloglevel = google::WARNING;
    FLAGS_minloglevel = google::INFO;

    if (argc < 2) {
        Usage();
        return 1;
    }

    std::string infile = argv[1];

    // MUST call this once to initial communications via cURL
    //cURLpp::Cleanup myCleanup;

    try {

        testOsrmClient();


        Timer starttime;

        CONFIG->set("plotDir", "./logs/");
        //CONFIG->set("osrmBaseUrl", "http://imaptools.com:5000/");
        CONFIG->dump("CONFIG");

       
        FeasableSolLoop tp(infile);
        tp.dump();

        return 0;

        DLOG(INFO) << "FeasableSol time: " << starttime.duration();
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
        DLOG(INFO) << "Total time: " << starttime.duration();

    }
    catch (const std::exception &e) {
        DLOG(FATAL) << e.what() << std::endl;
        return 1;
    }

    return 0;
}




