
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <deque>
#include <math.h>

#include "init_trash.h"
#include "prob_trash.h"
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
P.dump();
        std::string title;
        std::cout << "Problem '" << infile << "'loaded\n";
//        DumbSolution sols(P);
//        sols.insertConstruction();
//        sols.insertDumbInitialSolutions();
//        std::cout << "ENDING\n";
//        sols.dump();
        
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
