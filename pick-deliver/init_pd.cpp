#include <cmath>
#include "plot1.h"
#include "vehicle.h"
#include "init_pd.h"





void Init_pd::dumbConstruction() {
    Vehicle truck(depot,Q);
    fleet.empty();
        for (int i=0; i<getOrderCount()-1; i++) {
           truck.pushOrder(getOrder(i));
        }
    std::cout<<"pushOrder()----->";    truck.tau(); truck.dump();
    std::cout<<"\norder to be removed():"; ordersList[2].dump();
//    plot("testing1.png","with all nodes in the path");

    truck.removeOrder(2); std::cout<<"\nremoveOrder(2)->>>>"; truck.tau(); truck.dump();
//    plot("testing2.png"," with out order 2 in the path");
    truck.insert(datanodes[6],2); std::cout<<"\ninsert(datanodes[6],2)->>>"; truck.tau(); truck.dump();

//    plot("testing3.png"," wdded node 6 in the path");
    truck.move(2,4); std::cout<<"\nmove(2,4)->>>>"; truck.tau(); truck.dump();
/*
    plot("testing4.png","swaped the nodes 2 and 4");

    truck.move(4,2); std::cout<<"\nmove(4,2)->>>>"; truck.tau();
    truck.swap(2,5); std::cout<<"\nswap(5,2)->>>>"; truck.tau();
    truck.swapstops(2,5); std::cout<<"\nswapstops(2,5)->>>>"; truck.tau();
    truck.swapstops(6,7); std::cout<<"\nswapstops(6,7)->>>>"; truck.tau();
*/
    fleet.push_back(truck);
}

void Init_pd::dumbConstructionAndBestMoveForward() {
    Vehicle truck(depot,Q);
    fleet.empty();
    sortOrdersbyDistReverse();
    int bestI;
    int bestJ;
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushOrder(getOrder(i));
        }
    truck.findBetterForward(bestI, bestJ);
    truck.move(bestI,bestJ);
    fleet.push_back(truck);
};
     
void Init_pd::withSortedOrdersConstruction() {
    sortOrdersbyDist();
    dumbConstruction();
    sortOrdersbyId();
};

void Init_pd::dumbAndHillConstruction() {
    Vehicle truck(depot,Q);
    fleet.empty();
    sortOrdersbyDistReverse();
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushOrder(getOrder(i));
        }
    truck.hillClimbOpt();
    fleet.push_back(truck);
};

void Init_pd::deliveryBeforePickupConstruction() {
    Vehicle truck(depot,Q);
    fleet.empty();
        for (int i=0; i<getOrderCount(); i++) {
           truck.pushDelivery(getOrder(i));
           truck.pushPickup(getOrder(i));
        }
    truck.dump();
    fleet.push_back(truck);
};

void Init_pd::sequentialConstruction() {
    std::cout << "Enter Problem::sequentialConstruction\n";
    fleet.clear();
    Vehicle truck(depot,Q);
    Order order;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    sortOrdersbyDist();
    unOrders=ordersList;
    while (!unOrders.empty()) {
       truck.clean();  
       //std::cout<<"\n\n*******1 original orders"<<P.O.size()<<" wait orders "<< waitOrders.size()<<" unassigned Orders "<<unOrders.size()<<"\n";
       while (!unOrders.empty()) {
          order=unOrders.front();
          unOrders.pop_front();
          truck.pushOrder(order);
          truck.hillClimbOpt();     
          if (!truck.feasable()) {
                truck.removeOrder(order.getoid());
                waitOrders.push_back(order);
          }
       }
       fleet.push_back(truck);
       unOrders=waitOrders;
       waitOrders.clear();
     }
     dump();
}


void Init_pd::initialByOrderSolution() {
    int bppos, bdpos;
    int ppos, dpos;
    double actualcost, bestcost;
    fleet.clear();
    Order order;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    sortOrdersbyDistReverse();
    //unOrders=P.O;
    while (!unOrders.empty()) {
     //  Vehicle route(P);
       while (!unOrders.empty()) {
         order=unOrders.front();
          unOrders.pop_front();
          //route.pushOrder(order);
          //ppos=bppos=route.getppos(order.oid);
          //dpos=bdpos=route.getdpos(order.oid);
          actualcost=getcost();
          //bestcost=route.findBestCostBackForw(order.oid,bppos,bdpos); //can it come back with already tested for feasability
          if (bestcost<actualcost) {     //found a better place
             if (bppos<bdpos) {
                 //route.move(ppos,bppos);
                 //route.move(dpos,bdpos);
             }
          }
        /*  if (!route.feasable() ) {
                route.removeOrder(order);
                waitOrders.push_back(order);
          } */
       }
       //fleet.push_back(route);
       unOrders=waitOrders;
       waitOrders.clear();
     }
     dump();
}


void  Init_pd::initialFeasableSolution() {
    int bppos, bdpos;
    int ppos, dpos;
    double actualcost, bestcost;
    fleet.clear();
    Order order;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    //P.sortOrdersbyDistReverse();
    //unOrders=P.O;
    while (!unOrders.empty()) {
       //Vehicle route(P);
       while (!unOrders.empty()) {
         order=unOrders.front();
          unOrders.pop_front();
          //route.pushOrder(order);
          //ppos=bppos=route.getppos(order.oid);
          ////dpos=bdpos=route.getdpos(order.oid);
          actualcost=getcost();
          //bestcost=route.findBestCostBackForw(order.oid,bppos,bdpos); //can it come back with already tested for feasability
          if (bestcost<actualcost) {     //found a better place
             if (bppos<bdpos) {          
                 //route.move(ppos,bppos); 
                 //route.move(dpos,bdpos);
             }
          }
       /*   if (!route.feasable() ) {
                route.removeOrder(order);
                waitOrders.push_back(order);
          } */
       }      
       //fleet.push_back(route);
       unOrders=waitOrders;
       waitOrders.clear();
     }
     dump();
     //plotTau();

}



void Init_pd::computeCosts() {
    totalCost = 0.0;
    totalDistance = 0.0;
    for (int i=0; i<fleet.size(); i++) {
        totalCost += fleet[i].getcost();
        totalDistance += fleet[i].getduration();
    }
}

double Init_pd::getcost() {
    computeCosts();    // somewhere in the code the getcost returns 0 because the cost hant been computed
    return totalCost;
}

double Init_pd::getDistance() {
    computeCosts();
    return totalDistance;
}


void Init_pd::plot(std::string file,std::string title){
    std::vector<Dpnode> nodes;
    std::vector<int> pickups;
    std::vector<int> deliverys;
    std::vector<int> depots;

    for (int i=0; i<datanodes.size(); i++){
        nodes.push_back(datanodes[i]);
        if (datanodes[i].ispickup())
            pickups.push_back(datanodes[i].getnid());
        else if (datanodes[i].isdelivery())
            deliverys.push_back(datanodes[i].getnid());
        else if (datanodes[i].isdepot())
            depots.push_back(datanodes[i].getnid());
    }
    Plot1<Dpnode> plot( nodes );
    plot.setFile( file );
    plot.setTitle( title+".png" );
    plot.drawInit();
    for (int i=0; i<fleet.size(); i++) {
        plot.drawPath(fleet[i].getpath(), plot.makeColor(i), 1, false);
    }
    plot.drawPoints(pickups, 0x0000ff, 9, true);
    plot.drawPoints(depots, 0xff0000, 7, true);
    plot.drawPoints(deliverys, 0x00ff00, 5, true);
    plot.save();
    /* now a graph for each individual trucl */
    for (int i=0;i<fleet.size();i++) {
        fleet[i].plot(file,title,i);
    }
}


void Init_pd::tau() {
    std::cout<< "\nTau:" << std::endl;
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].tau();
    };
    std::cout<<"0\n";
}

void Init_pd::dumproutes()  {
    std::cout<< "\nVehicle:" << std::endl;
    for (int i=0; i<fleet.size(); i++) {
        std::cout<<"\n -----> Vehicle#"<<i<<"\n";
        fleet[i].dump();
    }
    tau();
}


void Init_pd::dump() {
    computeCosts();
    std::cout << "Solution: totalDistance: " << totalDistance
              << ", totalCost: " << totalCost
              << std::endl;
    tau();
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].dump();
    }
}


double Init_pd::getAverageRouteDurationLength() {
    double len = 0.0;
    int n = 0;
    for (int i=0; i<fleet.size(); i++) {
        //compact the fleet (i.e. eliminate from the list vehicles with no route  if (fleet[i].path.size() == 0) continue;
        fleet[i].evaluate();
        //if (fleet[i].updated) fleet[i].update();
        len += fleet[i].getduration();
        n++;
    }
    if (n == 0) return 0;
    return len/n;
}
