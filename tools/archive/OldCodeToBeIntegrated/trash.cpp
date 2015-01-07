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
#include <vector>
#include <math.h>

#include <stdio.h>

#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "feasableSol.h"
//#include "oneTruckAllNodesInit.h"



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

    try {
       
        FeasableSol tp(infile);
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




