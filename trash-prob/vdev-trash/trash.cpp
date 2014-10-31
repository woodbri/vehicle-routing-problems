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

#include "trashconfig.h"
#include "osrm.h"
#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "feasableSol.h"
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

        CONFIG->set("plotDir", "./logs/");
//        CONFIG->set("osrmBaseUrl", "http://imaptools.com:5000/");
        CONFIG->dump("CONFIG");
	//test that I can have a copy of the config, not that is usful but it cab be done
	TrashConfig configCopy;
	configCopy.dump("osrmBaseUrl");

       
        FeasableSol tp(infile);
        tp.setInitialValues();
        tp.dumpCostValues();

        TabuOpt ts(tp);
        ts.setMaxIteration(1);      // Added this to remove hardcoding
        ts.search();
        ts.dumpStats();

        Solution best = ts.getBestSolution();
        best.dumpCostValues();

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




