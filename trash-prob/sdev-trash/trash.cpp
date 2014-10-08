
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <stdio.h>

#include "trashconfig.h"
#include "osrm.h"
#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "feasableSol.h"
#include "tabusearch.h"



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

        CONFIG->plotDir = "./logs/";

        std::cout << "CONFIG: osrmBaseUrl: " << CONFIG->osrmBaseUrl << std::endl
                  << "            plotDir: " << CONFIG->plotDir << std::endl
                  << "       plotFontFile: " << CONFIG->plotFontFile << std::endl
                  << std::endl;
       
        FeasableSol tp(infile);
        //tp.dump();

        //const Solution sol = tp;

        TabuSearch ts(tp);
        ts.search();
        Solution best = ts.getBestSolution();
        best.dump();
        ts.dumpStats();

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}




