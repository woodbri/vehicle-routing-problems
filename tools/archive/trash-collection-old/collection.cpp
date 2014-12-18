
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "node.h"
#include "problem.h"


// create a static global variable for the problem
// this gets passed around as a reference in many of the objects
// that need to refer to it for data
static Problem P;

void Usage()
{
    std::cout << "Usage: collection in.txt\n";
}


int main (int argc, char **argv)
{
    if (argc < 2) {
        Usage();
        return 1;
    }

    char * infile = argv[1];

    try {
        std::string title;
        P.loadProblem(infile);
        std::cout << "Problem '" << infile << "'loaded\n";
        P.dump();


    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}

