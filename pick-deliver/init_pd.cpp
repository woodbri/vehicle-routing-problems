#include <cmath>
#include "plot.h"
#include "vehicle.h"
#include "init_pd.h"

// NON class functions for sorting
bool sortByOid(Order a, Order b)
{
    return a.oid < b.oid;
}

// Class functions

void Init_pd::dumbConstruction() {
    Vehicle truck;
    sortOrdersbyDistReverse();
        for (int i=0; i<getOrderCount(); i++) {
           truck.addOrder(getOrder(i));
        }
    truck.dump();
    fleet.push_back(truck);
};

void Init_pd::dumbConstructionAndBestMoveForward() {/*
    Vehicle dumbroute(P);
    int bestI;
    int bestJ;
        for (int i=0; i<P.getOrderCount(); i++) {
           dumbroute.addOrder(P.getOrder(i));
        }
    dumbroute.dumppath();
    dumbroute.findBetterForward(bestI, bestJ);
std::cout<<"best I"<<bestI<<", best J"<<bestJ<<"\n";
    dumbroute.move(bestI,bestJ);
    dumbroute.dumppath();
    
    fleet.push_back(dumbroute);
*/};
     
void Init_pd::withSortedOrdersConstruction() {
    sortOrdersbyDist();
    dumbConstruction();
};

void Init_pd::dumbAndHillConstruction() {/*
    Vehicle dumbroute(P);
    sortOrdersbyDistReverse();
        for (int i=0; i<getOrderCount(); i++) {
           dumbroute.addOrder(getOrder(i));
        }
    dumbroute.hillClimbOpt();
    dumbroute.dump();
    fleet.push_back(dumbroute);
*/};

void Init_pd::deliveryBeforePickupConstruction() {/*
    Vehicle dumbroute(P);
        for (int i=0; i<getOrderCount(); i++) {
           dumbroute.addDelivery(getOrder(i));
           dumbroute.addPickup(P.getOrder(i));
        }
    dumbroute.dump();
    fleet.push_back(dumbroute);
*/};

void Init_pd::sequentialConstruction() {
/*    // std::cout << "Enter Problem::sequentialConstruction\n";
    fleet.clear();
    Order order;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    sortOrdersbyDist();
    unOrders=P.O;
    while (!unOrders.empty()) {
       Vehicle route(P);
       std::cout<<"\n\n*******1 original orders"<<P.O.size()<<" wait orders "<< waitOrders.size()<<" unassigned Orders "<<unOrders.size()<<"\n";
       while (!unOrders.empty()) {
          order=unOrders.front();
          unOrders.pop_front();
          route.addOrder(order);
          route.hillClimbOpt();     
          if (!route.feasable()) {
                route.removeOrder(order);
                waitOrders.push_back(order);
          }
       }
       fleet.push_back(route);
       unOrders=waitOrders;
       waitOrders.clear();
     }
     dump();
*/}


void Init_pd::initialByOrderSolution() {
/*    int bppos, bdpos;
    int ppos, dpos;
    double actualcost, bestcost;
    fleet.clear();
    Order order;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    sortOrdersbyDistReverse();
    unOrders=P.O;
    while (!unOrders.empty()) {
       Vehicle route(P);
       while (!unOrders.empty()) {
         order=unOrders.front();
          unOrders.pop_front();
          route.addOrder(order);
          ppos=bppos=route.getppos(order.oid);
          dpos=bdpos=route.getdpos(order.oid);
          actualcost=getcost();
          bestcost=route.findBestCostBackForw(order.oid,bppos,bdpos); //can it come back with already tested for feasability
          if (bestcost<actualcost) {     //found a better place
             if (bppos<bdpos) {
                 route.move(ppos,bppos);
                 route.move(dpos,bdpos);
             }
          }
          if (!route.feasable() ) {
                route.removeOrder(order);
                waitOrders.push_back(order);
          }
       }
       fleet.push_back(route);
       unOrders=waitOrders;
       waitOrders.clear();
     }
     dump();*/
}


void  Init_pd::initialFeasableSolution() {/*
    int bppos, bdpos;
    int ppos, dpos;
    double actualcost, bestcost;
    fleet.clear();
    Order order;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    P.sortOrdersbyDistReverse();
    unOrders=P.O;
    while (!unOrders.empty()) {
       Vehicle route(P);
       while (!unOrders.empty()) {
         order=unOrders.front();
          unOrders.pop_front();
          route.addOrder(order);
          ppos=bppos=route.getppos(order.oid);
          dpos=bdpos=route.getdpos(order.oid);
          actualcost=getcost();
          bestcost=route.findBestCostBackForw(order.oid,bppos,bdpos); //can it come back with already tested for feasability
          if (bestcost<actualcost) {     //found a better place
             if (bppos<bdpos) {          
                 route.move(ppos,bppos); 
                 route.move(dpos,bdpos);
             }
          }
          if (!route.feasable() ) {
                route.removeOrder(order);
                waitOrders.push_back(order);
          }
       }      
       fleet.push_back(route);
       unOrders=waitOrders;
       waitOrders.clear();
     }
     dump();
     plotTau();
*/
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


void Init_pd::plot(){
    std::vector<double> x;
    std::vector<double> y;
    std::vector<int> label;
    std::vector<int> pcolor;
    std::vector<int> lcolor;
    int basecolor=10;
    for (int i=0; i<fleet.size(); i++) {
        fleet[i].plot(x,y,label,pcolor);
        for (int j=0; j<x.size(); j++) {
            if (label[j]==0) basecolor+=10;
            lcolor.push_back(basecolor);
        }
    }
    x.push_back(x[0]);
    y.push_back(y[0]);
    pcolor.push_back(pcolor[0]);
    lcolor.push_back(lcolor[0]);
    label.push_back(label[0]);

    Plot graph(x,y,pcolor,lcolor,label);
    graph.plot(false);
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