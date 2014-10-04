#include "solution.h"



void Solution::computeCosts() {
    totalCost = 0.0;
    totalDistance = 0.0;
    for (int i=0; i<fleet.size(); i++) {
        totalCost += fleet[i].getcost();
        totalDistance += fleet[i].getduration();
    }
}

double Solution::getCost() {
    computeCosts();    // somewhere in the code the getcost returns 0 because the cost hant been computed
    return totalCost;
}

double Solution::getDistance() {
    computeCosts();
    return totalDistance;
}





// this is a list of the node ids representing a vehicle route and 
// each vehicle is separated with a -1

std::string Solution::solutionAsText() const {
    std::stringstream ss;;
    const std::vector<int> sol = solutionAsVector();
    for (int i=0; i<sol.size(); i++) {
        if (i) ss << ",";
        ss << sol[i];
    }
//std::cout<<ss.str()<<"  Solution::solutionAsText() \n";
    return ss.str();
}

std::string Solution::solutionAsTextID() const {
    std::stringstream ss;; 
    const std::vector<int> sol = solutionAsVectorID();
    for (int i=0; i<sol.size(); i++) {
        if (i) ss << ",";
        ss << sol[i];
    }
    return ss.str();
}   
    

// create a vector of node ids representing a solution
// this can be used to save a compute solution while other changes
// are being tried on that solution and can be used with
// buildFleetFromSolution() to reconstruct the solution

std::vector<int>  Solution::solutionAsVectorID() const {
    std::vector<int> sol;
    sol.push_back(-1);

    for (int i=0; i<fleet.size(); i++) {
        if (fleet[i].size() == 0) continue;
        sol.push_back(fleet[i].getVid());
        sol.push_back(-1);
        for (int j=0; j<fleet[i].size(); j++) {
            sol.push_back(fleet[i][j].getid());
        }
        sol.push_back(fleet[i].getdumpSite().getid());
        sol.push_back(fleet[i].getdepot().getid());
        sol.push_back(-1);
    }
    return sol;
}

        
std::vector<int>  Solution::solutionAsVector() const {
    std::vector<int> sol;
    sol.push_back(-2);
    for (int i=0; i<fleet.size(); i++) {
        if (fleet[i].size() == 0) continue;
        sol.push_back(fleet[i].getVid());
        sol.push_back(-2);
        for (int j=0; j<fleet[i].size(); j++) {
            sol.push_back(fleet[i][j].getnid());
        }
        sol.push_back(fleet[i].getdumpSite().getnid());
        sol.push_back(fleet[i].getdepot().getnid());
        sol.push_back(-2);
    }
    return sol;
}


void Solution::plot(std::string file,std::string title){

    Plot<Trashnode> graph( datanodes );
    graph.setFile( file+".png" );
    graph.setTitle( datafile+": "+title );
    graph.drawInit();
    Prob_trash::plot(graph);
    for (int i=0; i<fleet.size(); i++) {
          fleet[i].plot(graph,i);
    }
    graph.save();

// a grpah for individual truck but with all nodes 
        
    for (int j=0;j<fleet.size();j++) {
        Plot<Trashnode> graph1( datanodes );
        std::stringstream convert;
        convert << j;
        std::string carnum = convert.str();

        graph1.setFile( file+"car"+carnum+".png" );
        graph1.setTitle( datafile+": "+title+" car #"+carnum );
        graph1.drawInit();
        Prob_trash::plot(graph1);
        fleet[j].plot(graph1,j);
        //graph1.drawPath(fleet[j].getpath(), graph1.makeColor(j*10), 1, false);
        graph1.save();
    }

//     now a graph for each individual truck 
    for (int i=0;i<fleet.size();i++) {
        fleet[i].plot(file,datafile+": "+title,i);
    }

}


void Solution::tau() {
    std::cout<< "\nTau:" << std::endl;
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].tau();
    };
    std::cout<<"0\n";
}

void Solution::dumproutes()  {
    std::cout<< "\nVehicle:" << std::endl;
    for (int i=0; i<fleet.size(); i++) {
        std::cout<<"\n -----> Vehicle#"<<i<<"\n";
        fleet[i].dump();
    }
    tau();
}


void Solution::dump() {
    computeCosts();
    tau();
    std::cout <<   " Total Distance: " << totalDistance
              << "\n     Total Cost: " << totalCost
              << std::endl;
    for (int i=0; i<fleet.size(); i++) {
        std::cout << "V" << i << " Total OSRM Time: "
                  << fleet[i].getTimeOSRM() << std::endl;
        fleet[i].getBackToDepot().dumpeval();
    }
}


double Solution::getAverageRouteDurationLength() {
    double len = 0.0;
    int n = 0;
    for (int i=0; i<fleet.size(); i++) {
      if (fleet[i].size()>0) {
        len += fleet[i].getduration();
        n++;
      }
    }
    if (n == 0) return 0;
    return len/n;
}


 Solution::Solution(const std::string &infile, const std::vector<int> &sol):Prob_trash(infile) {

    int nid,vid;
    Vehicle truck;
    Bucket unassigned = pickups;
    Bucket assigned;
    bool idSol=true;
    if (sol[0]==-2) idSol=false;

    fleet.clear();
    Bucket solPath;

    int i=1;
    while (i<sol.size()) {
        if (sol[i]<0 and sol[i+1]>0) break; //expected: vid -1
        vid = sol[i];
                
        //get the truck from the truks:
        for (int tr=0;tr<trucks.size();tr++) 
           if (trucks[tr].getVid()==vid){
              truck=trucks[tr];
              break;
           }

        i=i+2;
        solPath.clear();
        while (i<sol.size() and sol[i]>=0) {
   
           if (idSol) nid=pickups.getNidFromId(sol[i]);
           else nid=sol[i];

           solPath.push_back(datanodes[nid]);
           i++;
        }
solPath.dumpid("solPath");
        if (truck.e_setPath(solPath)) {
             fleet.push_back(truck);
             assigned+=solPath;
             unassigned-=solPath;
        }
        i++;
   };
   if (unassigned.size() or (assigned == pickups)) 
       std::cout<<"Something went wrong creating the solution\n";
};
        



