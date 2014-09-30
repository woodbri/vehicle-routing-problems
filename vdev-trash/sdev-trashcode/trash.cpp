
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
#include "sweep3.h"
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
       
        Sweep3 tp(infile);

        Solution sol1(infile,tp.solutionAsVector())  ; 
	assert(tp.solutionAsText()==sol1.solutionAsText());
	assert(tp.solutionAsTextID()==sol1.solutionAsTextID());
        Solution sol2(infile,tp.solutionAsVectorID())  ; 
	assert(tp.solutionAsText()==sol2.solutionAsText());
	assert(tp.solutionAsTextID()==sol2.solutionAsTextID());
        std::cout<<tp.solutionAsTextID()<<"\n";
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




