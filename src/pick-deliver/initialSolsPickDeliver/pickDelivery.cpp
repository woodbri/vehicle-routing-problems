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
#include <deque>
#include <math.h>

#include "init_pd.h"
#include "prob_pd.h"
#include "dumbsolution.h"


void Usage()
{
    std::cout << "Usage: tester in.txt\n";
}


int main (int argc, char **argv)
{
    if (argc < 2) {
        Usage();
        return 1;
    }

    char * infile = argv[1];
    
    try {   //ALL THESE ARE TESTS DONT NEED TO BE REAL SOLUTIONS

        Prob_pd P(infile);   //setting a new problem
        std::string title;
        std::cout << "Problem '" << infile << "'loaded\n";
        DumbSolution sols(P);
        sols.insertConstruction();
        sols.insertDumbInitialSolutions();
        std::cout << "ENDING\n";
        sols.dump();
        
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
