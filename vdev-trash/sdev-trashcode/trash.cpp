
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
#include "trashproblem.h"



void test_trash(std::string infile) ;
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

        test_trash(infile);

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}


//---------------------------------------------------------------
// run trash collection problem
//---------------------------------------------------------------
void test_trash(std::string infile) {

        TrashProblem tp(infile);

        //tp.loadproblem( infile );

        tp.dumpdataNodes();
        //tp.dumpDmatrix();
//        tp.dump();

/*
        std::cout << "\n----------- assignmentSweep3 -----------------------\n";
        tp.setRatio(0.9);
        tp.assignmentSweep3();
        tp.dump();

        tp.plot("p3.png", "assignmentSweep3", font);

        std::cout << "\n----------- doing 3-opt -----------------------\n";
        tp.opt_3opt();
        tp.dumpSummary();
        tp.plot("p5.png", "assignmentSweep3 - after 3opt", font);

        std::cout << "\n----------- doing 2-opt -----------------------\n";
        tp.opt_2opt();
        tp.dumpSummary();
        tp.plot("p4.png", "assignmentSweep3 - after 2opt", font);

        std::cout << "\n----------- doing or-opt -----------------------\n";
        tp.opt_or_opt();
        tp.dumpSummary();
        tp.plot("p6.png", "assignmentSweep3 - after or-opt", font);

        std::cout << "\n----------- doing 2-opt again -----------------\n";
        tp.opt_2opt();
        tp.dumpSummary();
        tp.plot("p7.png", "assignmentSweep3 - after 2opt again", font);

        std::cout << "\n----------- doing pathOptimize ---------------------\n";
        tp.optimize();
        tp.dump();
        tp.plot("p7.png", "assignmentSweep2 - after optimize", font);
*/

}


