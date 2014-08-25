
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
    
    try {
        Prob_pd P;   //setting a new problem
        std::string title;
        P.loadProblem(infile); //load problem
        std::cout << "Problem '" << infile << "'loaded\n";
        std::cout << "\n\n\n**SOLUTION: dumbConstruction\n";
        Init_pd S(P);  S.setweights(1,1,1); 
        S.dumbConstruction();  P.dump(); S.dump(); 
        S.plot("test1.png","Dumb construction");

//        std::cout << "\n\n\n**SOLUTION: deliveryBeforePickupConstruction\n";
//        Init_pd S1(P);  

//        S1.setweights(1,1,1); 
//        P.dump(); 
//        S1.deliveryBeforePickupConstruction();  
//        S1.dump(); 
//        S1.plot("test2.png","Dumb construction & best move forward");
/*        std::cout << "\n\n\n**SOLUTION: dumbConstruction and bestmove forward\n";
        Init_pd S2(P);  S2.setweights(1,1,1); 
        S2.dumbConstructionAndBestMoveForward();  P.dump(); S2.dump(); S2.plot("test2.png","Dumb construction & best move forward");

   */     //S.initialFeasableSolution();
/*
        std::cout << "\n\n\n******************Y************Solution: initial no hill Construction \n";
        S.initialNoHillConstruction();
        Plot plot4(S);
        title = (std::string)infile+"-initialNoHillSolution.png";
        plot4.out(title, true, 800, 800, (char*)title.c_str());
S.R.clear();
        std::cout << "\n\n\n******************************Solution: delivery befor pickup, The NO NO solution\n";
        S.deliveryBeforePickupConstruction();
        Plot plot3(S);
        title = (std::string)infile+"-NoNoSolution.png";
        plot3.out(title, true, 800, 800, (char*)title.c_str());

S.R.clear();
        std::cout << "\n\n\n******************************Solution: dumb solution \n";
        S.dumbConstruction();
//        Plot plot(S);
//        title = (std::string)infile+"-DumbSolution.png";
//        plot.out(title, true, 800, 800, (char*)title.c_str());
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
