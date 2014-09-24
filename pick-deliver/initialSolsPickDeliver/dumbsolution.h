#ifndef DUMBSOLUTION_H
#define DUMBSOLUTION_H
#include <deque>
#include "prob_pd.h" 
#include "init_pd.h" 
#include "solution.h" 
#include "testconstruction.h" 

class DumbSolution {

private:
      Prob_pd problem;
      std::deque<Solution> taus;
public:

      DumbSolution(const Prob_pd &P):problem(P){};
      void push_back(const Solution solution) {
            taus.push_back(solution);
      };

      
      void insertConstruction() {
std::cout<<"Enter insertConstruction\n";
          Init_pd test(problem);
std::cout<<"Enter insertConstruction 1\n";
          taus.push_back(test);
std::cout<<"Enter insertConstruction 2\n";
dump();
      }


      void insertDumbInitialSolutions() {
          TestConstruction test(problem);
          test.dumbConstruction();
          taus.push_back(test);
/*          test.dumbConstructionAndBestMoveForward();
          taus.push_back(test);
          test.withSortedOrdersConstruction();
          taus.push_back(test);
          test.dumbAndHillConstruction();
          taus.push_back(test);
          test.deliveryBeforePickupConstruction();
          taus.push_back(test);
*/
      };
      void dump(int i) {
              std::cout<<"\n***** SOLUTION #"<<i<<"********\n";
              taus[i].dump();
              std::cout<<"\n--------------------------------";
      };
      void dump() {
          for (int i=0;i<taus.size();i++) {
              dump(i);
          }
      };


};
#endif      
