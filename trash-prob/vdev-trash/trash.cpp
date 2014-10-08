
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
#include "truckManyVisitsDump.h"
//#include "feasableSol.h"
#include "oneTruckAllNodesInit.h"



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
       
        OneTruckAllNodesInit   sol1(infile);
        TruckManyVisitsDump sol2(sol1);
        std::cout<<"\n"<<sol2.solutionAsTextID();
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




