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

#ifdef DOVRPLOG
#include "logger.h"
//#include "glog/utilities.h"
#endif

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif



#include "trashconfig.h"
#include "feasableSolLoop.h"
#include "tabuopt.h"


void Usage()
{
  std::cout << "Usage: trash file (no extension)\n";
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

    http://google-glog.googlecode.com/svn/trunk/doc/glog.html

    GLOG_logtostderr=1 ./bin/trash ...  // to get output to terminal
*/

int main(int argc, char **argv)
{

#ifdef DOVRPLOG

  if ( not google::IsGoogleLoggingInitialized() ) {
    FLAGS_log_dir = "./logs/";
    google::InitGoogleLogging( "vrp_trash_collection" );
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::FATAL;
    FLAGS_minloglevel = google::INFO;
  }

#endif

//#define MAKETEST
#ifdef MAKETEST

//  expected output when ran as:
//  sudo -u postgres bin/maketest
//
//  i: 0, time: 0
//  i: 1, time: 1.18333
//  i: 2, time: 1.81667

    osrmi->clear();
    osrmi->useOsrm( true );
    osrmi->addViaPoint( -34.905113,-56.157043 );
    osrmi->addViaPoint( -34.906807,-56.158463 );
    osrmi->addViaPoint( -34.9076,-56.157028 );
    if ( osrmi->getOsrmViaroute() ) {

        std::deque<double> times;
        if ( osrmi->getOsrmTimes( times ) ) {
            std::cout << "Times:" << std::endl;
            for (int i=0; i<times.size(); i++)
                std::cout << "i: " << i << ", time: " << times[i] << std::endl;
        }
        else {
            std::cout << "getOsrmTimes Failed!" << std::endl;
            return 1;
        }

        std::deque<std::string> hints;
        if ( osrmi->getOsrmHints( hints ) ) {
            std::cout << "Hints:" << std::endl;
            for (int i=0; i<hints.size(); i++)
                std::cout << "i: " << i << ", hint: " << hints[i] << std::endl;
        }
        else {
            std::cout << "getOsrmHints Failed!" << std::endl;
            return 1;
        }

        std::deque<std::string> names;
        if ( osrmi->getOsrmStreetNames( names ) ) {
            std::cout << "StreetNames:" << std::endl;
            for (int i=0; i<names.size(); i++)
                std::cout << "i: " << i << ", name: " << names[i] << std::endl;
        }
        else {
            std::cout << "getOsrmStreetNames Failed!" << std::endl;
            return 1;
        }
    }
    else {
        std::cout << "getOsrmViaroute Failed!" << std::endl;
        return 1;
    }

    return 0;
#endif

  if (argc < 2) {
    Usage();
    return 1;
  }

  std::string infile = argv[1];

  try {
#ifdef DOSTATS
    Timer starttime;
#endif

#ifdef VRPMINTRACE
    //CONFIG->dump("CONFIG");
#endif

    FeasableSolLoop tp(infile);


#ifdef VRPMINTRACE
    tp.dumpCostValues();
#ifdef DOSTATS
    DLOG(INFO) << "FeasableSol time: " << starttime.duration();
#endif
#endif

#ifdef DOSTATS
    STATS->set("FeasableSol time", starttime.duration());
#endif

    tp.setInitialValues();
    tp.computeCosts();
#ifdef DOSTATS
    STATS->set("Initial cost", tp.getCost());
    STATS->set("Node count", tp.getNodeCount());
    STATS->set("Vehicle count", tp.getFleetSize());
    Timer searchtime;
#endif

    int iteration = 3;
    TabuOpt ts(tp, iteration);

#ifdef DOSTATS
    STATS->set("Search time", searchtime.duration());
#endif

    Solution best = ts.getBestSolution();
    best.computeCosts();

#ifdef DOSTATS
    STATS->set("Total time", starttime.duration());
#endif

#ifdef VRPMINTRACE
    best.dumpCostValues();
    best.tau();
#endif

#ifdef DOSTATS
    STATS->set("best cost", best.getCost());
    STATS->set("Best distance", best.getDistance());

    STATS->dump("Final");
#endif

    best.dumpSolutionForPg();
    twc->cleanUp();

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  return 0;
}




