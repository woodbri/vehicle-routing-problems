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

#ifdef LOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif



#include "trashconfig.h"
#include "feasableSol.h"
#include "tabuopt.h"


void Usage() {
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

int main(int argc, char **argv) {

    #ifdef DOVRPLOG
        if ( not google::IsGoogleLoggingInitialized() ) {                                                                                                                                                                        
            FLAGS_log_dir = "./logs/";
            google::InitGoogleLogging( "vrp_trash_collection" );
            FLAGS_logtostderr = 0;
            FLAGS_stderrthreshold = google::FATAL;                                                                                                                                                                               
            FLAGS_minloglevel = google::INFO;
        }                                                                                                                                                                                                                        
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
        CONFIG->dump("CONFIG");
        #endif
 
        FeasableSol tp(infile);
	

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

	int iteration=1;
        TabuOpt ts(tp,iteration);

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

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




