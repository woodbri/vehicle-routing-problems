
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "Node.h"
#include "Order.h"
#include "Problem.h"
#include "Route.h"
#include "Solution.h"
#include "Plot.h"
#include "TabuSearch.h"


// create a static global variable for the problem
// this gets passed around as a reference in many of the objects
// that need to refer to it for data
static Problem P;

void Usage()
{
    std::cout << "Usage: vrpdptw in.txt\n";
}


int main (int argc, char **argv)
{
    if (argc < 2) {
        Usage();
        return 1;
    }

    char * infile = argv[1];

    try {
        P.loadProblem(infile);
        std::cout << "Problem '" << infile << "'loaded\n";
        P.dump();

        Solution S(P);
        S.sequentialConstruction();
        //S.initialConstruction();
        S.computeCosts();
        std::cout << "Initial Solution: SCost: " << S.getCost() << std::endl;
        S.dump();

#if 1
        TabuSearch TS(S);
        TS.debugTabu = true;
        TS.debugPlots = false;
        Solution B = TS.solve();

        std::cout << "TabuSearch Results (1)" << std::endl;
        B.dump();

#if 0
        TabuSearch TS2(B);
        Solution C = TS2.solve();

        std::cout << "TabuSearch Results (2)" << std::endl;
        C.dump();
#endif
#else
        Solution B = S;
        for (int i=0; i<2; i++){
            TabuSearch TS(B);
            B = TS.solve();

            std::cout << "TabuSearch Results (" << i+1 << ")" << std::endl;;
            B.dump();
        }
#endif

        Plot plot(B);
        std::string title = "vrpdptw.png Final";
        plot.out("vrpdptw.png", true, 800, 800, (char*)title.c_str());
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
