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


void Solution::plot(std::string file,std::string title){

    Plot<Dpnode> graph( datanodes.Path() );
    graph.setFile( file+".png" );
    graph.setTitle( datafile+": "+title );
    graph.drawInit();

    for (int i=0; i<datanodes.size(); i++){
        if (datanodes[i].ispickup())  {
             graph.drawPoint(datanodes[i], 0x0000ff, 9, true);
        } else if (datanodes[i].isdelivery()) {
             graph.drawPoint(datanodes[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(datanodes[i], 0xff0000, 7, true);
        }
    }
    for (int i=0; i<fleet.size(); i++) {
        graph.drawPath(fleet[i].getpath(), graph.makeColor(i*10), 1, false);
    }
    graph.save();

/* a grpah for individual truck but with all nodes */
        
    for (int j=0;j<fleet.size();j++) {
        Plot<Dpnode> graph1( datanodes.Path() );
        std::stringstream convert;
        convert << j;
        std::string carnum = convert.str();

        graph1.setFile( file+"car"+carnum+".png" );
        graph1.setTitle( datafile+": "+title+" car #"+carnum );
        graph1.drawInit();

        for (int i=0; i<datanodes.size(); i++){
            if (datanodes[i].ispickup())  {
                 graph1.drawPoint(datanodes[i], 0x0000ff, 9, true);
          } else if (datanodes[i].isdelivery()) {
                 graph1.drawPoint(datanodes[i], 0x00ff00, 5, true);
          } else  {
                 graph1.drawPoint(datanodes[i], 0xff0000, 7, true);
          }
        }  
        graph1.drawPath(fleet[j].getpath(), graph1.makeColor(j*10), 1, false);
        graph1.save();
    }

    


/*     now a graph for each individual trucl */
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
