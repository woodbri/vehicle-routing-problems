
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
    
    try {   //ALL THESE ARE TESTS DONT NEED TO BE REAL SOLUTIONS

        Prob_pd P;   //setting a new problem
        std::string title;
        P.loadProblem(infile); //load problem
        std::cout << "Problem '" << infile << "'loaded\n";
//        P.dump();

/*
        std::cout << "\n\n\n**SOLUTION: Initial feasable Construction\n";
        Init_pd S5(P);  S5.setweights(1,1,1); 
        S5.initialFeasableSolution();  S5.dump(); S5.tau(); S5.plot("initialFeasableSolution","Initial Feasable Solution");
*/
        std::cout << "\n\n\n**SOLUTION: Secuential Construction\n";
        Init_pd S4(P);  S4.setweights(1,1,1); 
        S4.seqConst();  S4.tau();/* S4.dump();*/  S4.plot("seqConst","SequentialConstruction");
        //S4.orderConstraintConstruction();  S4.tau();/* S4.dump();*/  S4.plot("seqConst","SequentialConstruction");

/*

        std::cout << "\n\n\n**SOLUTION: dumbConstruction\n";
        Init_pd S(P);  S.setweights(1,1,1); 
        S.dumbConstruction(); S.tau(); S.plot("dumbConstruction","Dumb construction");

        std::cout << "\n\n\n**SOLUTION: deliveryBeforePickupConstruction\n";
        Init_pd S1(P);  S1.setweights(1,1,1); 
        S1.deliveryBeforePickupConstruction(); S1.tau(); S1.plot("DeliverybeforePickup","ideliveryBeforePickup");


        std::cout << "\n\n\n**SOLUTION: dumbConstruction and bestmove forward\n";
        Init_pd S2(P);  S2.setweights(1,1,1); 
        S2.dumbConstructionAndBestMoveForward();  S2.tau(); S2.plot("dumbConstructionAndBestMoveForward","Dumb construction & best move forward");
        std::cout << "\n\n\n**SOLUTION: withSortedOrdersConstruction\n";
        Init_pd S3(P);  S3.setweights(1,1,1); 
        S3.withSortedOrdersConstruction();  S3.tau(); S3.plot("withSortedOrdersConstruction","withSortedOrdersConstruction");

*/
        //S.initialFeasableSolution();
/*
        std::cout << "\n\n\n******************Y************Solution: initial no hill Construction \n";
        S.initialNoHillConstruction();
        Plot plot4(S);
        title = (std::string)infile+"-initialNoHillSolution.png";
        plot4.out(title, true, 800, 800, (char*)title.c_str());
S.R.clear();
        std::cout << "\n\n\n******************************Solution: dumb and hill opt \n";
        S.dumbAndHillConstruction();
//        Plot plot5(S);
//        title = (std::string)infile+"-SortedSolution.png";
//        plot5.out(title, true, 800, 800, (char*)title.c_str());


S.R.clear();
        std::cout << "\n\n\n******************************Solution: sorted orders by distance solution \n";
        S.withSortedOrdersConstruction();
        Plot plot1(S);
        title = (std::string)infile+"-SortedSolution.png";
        plot1.out(title, true, 800, 800, (char*)title.c_str());

S.R.clear();


        std::cout << "\n\n\n******************************Solution: dumb + sorted  \n";
        S.dumbConstruction();  //dumb
        S.withSortedOrdersConstruction(); //sort
        Plot plot2(S);
        title = (std::string)infile+"-Dumb+SortedSolution.png";
        plot2.out(title, true, 800, 800, (char*)title.c_str());
*/



//        S.dump();
          
//        S.sequentialConstruction();

        //S.initialConstruction();
//        S.computeCosts();
//        std::cout << "Initial Solution: SCost: " << S.getCost() << std::endl;
//        S.dump();
//        P.dump();

/*
//#id 1
        TabuSearch TS(S);
        TS.debugTabu = true;
        TS.debugPlots = false;
        Solution B = TS.solve();

        std::cout << "TabuSearch Results (1)" << std::endl;
        B.dump();

//#if 0
        TabuSearch TS2(B);
        Solution C = TS2.solve();

        std::cout << "TabuSearch Results (2)" << std::endl;
        C.dump();
//#endif
//#else
        Solution B = S;
        for (int i=0; i<2; i++){
            TabuSearch TS(B);
            B = TS.solve();

            std::cout << "TabuSearch Results (" << i+1 << ")" << std::endl;;
            B.dump();
        }
//#endif

        Plot plot(B);
        std::string title = "vrpdptw.png Final";
        plot.out("vrpdptw.png", true, 800, 800, (char*)title.c_str());
*/
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
