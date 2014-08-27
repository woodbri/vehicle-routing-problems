#include <cmath>
#include "plot1.h"
#include "vehicle.h"
#include "init_pd.h"

/* prerequisites 
 nodes sorted is based on amount of twc
 orders were made based on TWC 

*/
void Init_pd::seqConst() {
std::cout << "Enter Problem::seqConstruction\n";
    fleet.clear();
    Vehicle truck(depot,Q);
    Order order;
    std::deque<Order> unOrders;   //orders not being used
    std::deque<Order> waitOrders; //orders didnt fit in current route
    unOrders=ordersList;          //all orders arent in a route
    while (!unOrders.empty()) {   
       truck.clean();                           //create a new truck for a route
       //findrSeed(unOrders)   			//we are free to find an appropiate seed for the route
       while (!unOrders.empty()) {  
          /* asumption 0 n1 n2  are in the route
              que estan restringidos ya que se esogió como semilla aquel cuya cant de TWCn2 es la mayor
              por lo que de la cubeta de ordenes necesito buscar para cada orden(i) the tiene pick y deliver
                como pick esta restringida por deliver aka pick va antes que deliver, tengo que ver primero
		por deliver.
		para el deliver de la orden i
		recorro la ruta: 
			 0 	n1 	n2   	deli 	compatibleIAJ (n2, deli)     = no     // puede ir despues de n2??? (no creo)
			 0 	n1 	deli 	n2 	compatibleIAJ (n1, deli, n2) = si     //aka puede ir despues de n1 y antes de n2???
							compatibleIJ  (0, deli)      = si
                ahi se queda

		lo mismo se hace para pick pero del 1 al ultimo en la ruta
			 0 	pick	n1 	deli	n2   	compatibleIAJ (0, pick, n1)     = si  (a fuerzas ya que si no no se podria insertar la orden
								compatibleIJ (pick, deli)       = si  (a fuerza porque es su pick)
								compatibleIJ (pick, n2)         = si  ahi se queda

		cuando  compatibleIJ (pick, n2)  como no hay manera de que pick vaya antes que un nodo que esta a su derecha y n2 esta a su derecha esa orden
		no es posible insertarla y ademas en este ejemplo deli esta antes que n2, entonces no hay manera de insertar esta orden dentro de la ruta.


		tambien podria ser que empieze tratando de ver si wdeliver puede entrar en esa ruta

		sea la ruta:   0 n1 n2 n3 ...na p nb .....  nz      
                para los nodos 0....na  compatibleIJ( ni , p) donde ni va del nodo 0 al na
                para los nodos nb....nz  compatibleIJ(p , nj)   donde nj va del nodo nb al nodo nz

                lo mismo para deliver
		sea la ruta:   0 n1 n2 n3 ...na d nb .....  nz      
                para los nodos 0....na  compatibleIJ( ni , d) donde ni va del nodo 0 al na
                para los nodos nb....nz  compatibleIJ(d , nj)   donde nj va del nodo nb al nodo nz
		
		con un NO y solo con un NO, entonces en esa posicion no se puede

			 0 	n1 	pick	n2   	deli   	aka compatibleIJ (n2, deli)     = si
			 0 	n1 	n2   	pick	deli   	aka compatibleIJ (n2, deli)     = si
			 0 	n1 	n2      deli	pick   	aka compatibleIJ (n2, deli)     = si
		por ejemplo en la primera linea
													  "cuando no causa problemas" comparo con la mejor ruta hasta el momento para esa orden
													  y si es mejor entonces la guardo  como la mejor ruta
	  */
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
}





void Init_pd::dumbConstruction() {
    Vehicle truck(depot,Q);
    fleet.empty();
        for (int i=0; i<2; i++) {
           truck.pushOrder(getOrder(i));
        }
   fleet.push_back(truck);
   Vehicle car(depot,Q);
   for (int i=2; i<getOrderCount(); i++) {
           car.pushOrder(getOrder(i));
   }
   fleet.push_back(car);  
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
    sortOrdersbyIdReverse();
    dumbConstruction();
    //sortOrdersbyDist();
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
    fleet.push_back(truck);
};

void Init_pd::sequentialConstruction() {
std::cout << "Enter Problem::sequentialConstruction\n";
    fleet.clear();
    Vehicle truck(depot,Q);
    Order order;
    Order lastOrder;
    std::deque<Order> unOrders;
    std::deque<Order> waitOrders;
    unOrders=ordersList;
    while (!unOrders.empty()) {
       truck.clean();  
       while (!unOrders.empty()) {
          order=unOrders.front();
          unOrders.pop_front();
          truck.insertPickup(order,1);
          truck.pushDelivery(order);
          truck.hillClimbOpt();     
          if (!truck.feasable()) {
                truck.removeOrder(order.getoid());
                waitOrders.push_back(order);
          } else {
                lastOrder=order;
          }
       }
       fleet.push_back(truck);
       unOrders=waitOrders;
       waitOrders.clear();
     }
}


void Init_pd::initialByOrderSolution() {
    int bppos, bdpos;
    int ppos, dpos;
    double actualcost, bestcost;
    fleet.clear();
    Order order;
    std::deque<Order> clientBucket;
    std::deque<Order> waitOrders;
    sortOrdersbyDistReverse();
    clientBucket=ordersList;
    while (!clientBucket.empty()) {        //are there any unrouted customers
       Vehicle route(depot,Q);             // initialize tour  
       while (!clientBucket.empty()) {     //are there any unrouted customers?
          order=clientBucket.front();          
          clientBucket.pop_front();
          route.pushOrder(order);          //initialize tour with seed customer
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
       clientBucket=waitOrders;
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
//    sortOrdersbyDistReverse();
    unOrders=ordersList;
    while (!unOrders.empty()) {
       Vehicle route(depot,Q);
       while (!unOrders.empty()) {
          order=unOrders.front();
          unOrders.pop_front();
          route.pushOrder(order);
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

/*
    Twpath<Dpnode> trace=path;
    trace.push_back(backToDepot);

    // cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber *
    std::stringstream convert;
    convert << carnumber;
    std::string carnum = convert.str();
    std::string extra=file+"vehicle"+carnum ;
*/
    Plot1<Dpnode> graph( datanodes );
    graph.setFile( file+".png" );
    graph.setTitle( title );
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
        Plot1<Dpnode> graph1( datanodes );
        std::stringstream convert;
        convert << j;
        std::string carnum = convert.str();

        graph1.setFile( file+"car"+carnum+".png" );
        graph1.setTitle( title+" car #"+carnum );
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
      if (fleet[i].size()>0) {
        len += fleet[i].getduration();
        n++;
      }
    }
    if (n == 0) return 0;
    return len/n;
}
