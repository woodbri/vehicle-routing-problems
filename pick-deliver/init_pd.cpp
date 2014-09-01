#include <cmath>
#include "plot1.h"
#include "vehicle.h"
#include "compatible.h"
#include "init_pd.h"

/* prerequisites 
 nodes sorted is based on amount of twc
 orders were made based on TWC 

*/

Order Init_pd::getOrderData(int nodeId, int &pick, int &deliver){
    Order order;
    Dpnode node=twc.getNode(nodeId);
    order=  ordersList[ node.getoid() ];
    pick= order.getPickup().getnid();
    deliver= order.getDelivery().getnid();
    return order;
}

bool Init_pd::inPending(const Vehicle pend, int nodeId) {
    for (int i=0; i<pend.size();i++) {
         if (pend[i].getnid()==nodeId ) return true;
    };
    return false;
}

int Init_pd::compatibleWithPending(const Vehicle &pend, int fromId, int toId) {
    int notwithID = -1;
std::cout<<"notwithID ENTERING"<<notwithID;
    for (int i=0; i<pend.size();i++) {
         if (twc.isCompatibleIJ( toId, pend[i].getnid() )) {}
         else notwithID = pend[i].getnid();
std::cout<<"notwithID"<<notwithID;
    };
std::cout<<"notwithID EXITING"<<notwithID;
    return notwithID;
}

void Init_pd::insert(Vechicle &truck, int nodeId,Bucket &nodes) {
       Dpnode nodeToInsert=twc.getNode(bestId);
       truck.push_back(nodeToInsert);  //add the node to the truck
       nodes.removeNode(nodeId);                // remove the node from the nodes bucket
       twc.maskVertical(nodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)
}

void Int::removeIncompatible(int fromId,Bucke &nodes, Bucket &incomatible);
       int currId;
       Order =  order;
       int i = 0;
       while (i<nodes.size()) {
           currId=nodes[i].getnid();
           order =getOrderData(currId,pickId,deliverId);
           if (isCompatibleIJ(fromId,pickId) and isCompatible(fromId,deliverId) i++;
           else { // either pick or delivery are incompatible
                
       }


void Init_pd::paperConst(Vehicle &truck, Bucket &nodes, Bucket &incompatible, Bucket &pending) {
        int lastNodeId = truck[truck.size()-1].getnid();
        int insertedId;
        if (nodes.empty()) return; //no more compatible nodes
        bestId=twc.getBestCompatible(lastNodeId);
        if (bestId==-1) { std::cout<<"\nNever should I reach here "; break; };                    //no compatible node, have to leave //what with the other nodes???
        order =getOrderData(bestId,pickId,deliverId);
        if (bestId==pickId) {
            // pick & deliver are compatible to the last node
                insert(truck,pickId,nodes);
                pending.push_back(order.getDelivery());
                insertedId=pickId;
        } else { // best compatible is delivery
            if (inPending(pending,bestId)) { // pickup already in the route
                insert(truck,deliverId,nodes);
                pendingDeliveries.removeNode(bestId);
                insertedId=deliverId;
            } else {   //pickup is not in the route
                insert(truck,pickId,nodes);
                pending.push_back(order.getDelivery());
                insertedId=pickId;
            }
        }
        //move incompatible from nodes bucket
        
}    





// hanndling 2 things the nodes position in the original table and the nodes id 
void Init_pd::seqConst() {
std::cout << "Enter Problem::seqConst\n";
/* I already have the twc table ready */

    Vehicle picks,deliver,depts,nodes;
    Vehicle pendingDeliveries;   // delivery nodes not inserted yet 
    Dpnode nodeToInsert;
    Order order;
    nodes=originalnodes;
    for (int i=0;i<nodes.size();i++) {
         if (nodes.getnode(i).ispickup())  picks.push_back(originalnodes.getnode(i));
         if (nodes.getnode(i).isdelivery()) deliver.push_back(originalnodes.getnode(i));
         if (nodes.getnode(i).isdepot())  depts.push_back(originalnodes.getnode(i));
    }
std::cout<<"\n picks= ";
picks.tau();
std::cout<<"\n depts= ";
depts.tau();
std::cout<<"\n deliver= ";
deliver.tau();
    int pickups = twc.setSubset(picks);
    int deliverys = twc.setSubset(deliver);
    int depots = twc.setSubset(depts); //probably as many depots as the problem has cars
    Vehicle truck(depot,Q);
    int lastNodeId,lastNodePos, bestPos,bestId,actualCompat,deliveryId,pendingId,pickId,deliverId,notcompatWith;

    for (int i=0;i<ordersList.size();i++)
        twc.setIncompatible(ordersList[i].getdid(),ordersList[i].getpid());
twc.dump();
int k=0;
    while (k<20 and  nodes.hasNodes()) {   
    truck.clean();             //create a new truck for a route (depot has being inserted as first node)
    lastNodeId=depot.getnid(); 			// I work with nids at this levle
    nodes.removeNode(lastNodeId);                // remove the node from the nodes list
    twc.recreateRowColumn( lastNodeId );
    
    while ( k< 20 and nodes.hasNodes()) {   
k++;
std::cout<<"\n********************CYCLE=****************************** "<<k<<"\n";
std::cout<<"\nfinding best from lastNodeId= "<<lastNodeId;

        bestId=twc.getBestCompatible(lastNodeId) ;      //making sure I get a pickup?? first if its a depot I insert pickup first???? (is it a seed????)

        if (bestId==-1) { std::cout<<"\nENDING****************************** "<<k<<"\n"; break; };                    //no compatible node, have to leave //what with the other nodes???

std::cout<<"\nbestId= "<<bestId;
        order =getOrderData(bestId,pickId,deliverId);
//        if (bestId==deliverId) bestId=pickId;
        if (inPending(pendingDeliveries,bestId)) {
std::cout<<"\nInserting a node that was in pending   aka. bestID is delivery";
            nodeToInsert=twc.getNode(bestId);
	    //add the node to truck
            truck.push_back(nodeToInsert);  //add the node to the truck
            lastNodeId=nodeToInsert.getnid();
            //delete node from pending
            pendingDeliveries.removeNode(lastNodeId);
            nodes.removeNode(lastNodeId);                // remove the node from the nodes list
            twc.maskVertical(lastNodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)
            continue;
        }
        else
std::cout<<"\nNot in pending Best Id";

// not in pending
std::cout<<"\nGOING TO notcompatwith= "<<notcompatWith;
        notcompatWith=compatibleWithPending(pendingDeliveries, lastNodeId,  bestId);
std::cout<<"\nnotcompatwith= "<<notcompatWith;

        if (bestId==pickId) {
              notcompatWith=compatibleWithPending(pendingDeliveries, lastNodeId,  bestId);
              if (notcompatWith!=-1) { 
std::cout<<"\nInserting a node that was in pending   aka. bestID is delivery becasue the chosen node was incompatible with this node";
                   bestId=notcompatWith;
                   nodeToInsert=twc.getNode(bestId);
                   //add the node to truck
                   truck.push_back(nodeToInsert);  //add the node to the truck
                   lastNodeId=nodeToInsert.getnid();
                   //delete node from pending
                   pendingDeliveries.removeNode(lastNodeId);
                   nodes.removeNode(lastNodeId);                // remove the node from the nodes list
                   twc.maskVertical(lastNodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)
                   continue;
               } else {
std::cout<<"\nInserting a node that is a pickup that all pending are compatible";
                   bestId=bestId;
                   nodeToInsert=twc.getNode(bestId);
                   //add the node to truck
                   truck.push_back(nodeToInsert);  //add the node to the truck
                   lastNodeId=nodeToInsert.getnid();
                   //delete node from pending
                   //pendingDeliveries.removeNode(lastNodeId);
                   nodes.removeNode(lastNodeId);                // remove the node from the nodes list
                   twc.maskVertical(lastNodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)
std::cout<<"\n need to update de pending deliveries (becasue was a pickup)";
                   pendingDeliveries.push_back (  order.getDelivery() );
                   continue;
               }
         } 



std::cout<<"\nCaso especial, es un pickup y hay que meter su delivery primero falta corregir el codigo";
        
	actualCompat=twc.compatibleIJ(lastNodeId,bestId);
        nodeToInsert=twc.getNode(bestId);
        if (pendingDeliveries.hasNodes()) {
                for (int j=0; j<pendingDeliveries.size();j++) {
                    deliveryId=pendingDeliveries.getnid(j); 
std::cout<<"\ndeliveryId= "<<deliveryId;
                    if (deliveryId==bestId) {break;}  // the best is in the delivery list
		    if ( actualCompat<twc.compatibleIJ(lastNodeId,deliveryId) ) {
std::cout<<"\nbetter to insert a delivery";
                        nodeToInsert=pendingDeliveries.getnode(j) ;
                        bestId=pendingDeliveries.getnid(j);
                        break;
                    };
                 };
        };

        twc.maskHorizontal(lastNodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)

std::cout<<"\n NodetoInsert= "; nodeToInsert.dump();

        truck.push_back(nodeToInsert);  //add the node to the truck
        lastNodeId=nodeToInsert.getnid();

std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";

        if (nodeToInsert.ispickup()) {
std::cout<<"\norder to where it belongs= "<<nodeToInsert.getoid(); std::cout<<"\n";

                 pendingDeliveries.push_back (  ordersList[ nodeToInsert.getoid() ].getDelivery() );
        } else {
std::cout<<"\n removing delivery node = ";
std::cout<<"\n lastNodeId= "<<lastNodeId;
std::cout<<"\n pending= "; pendingDeliveries.tau(); std::cout<<"\n";
                 pendingDeliveries.removeNode(lastNodeId);
std::cout<<"\n pending= "; pendingDeliveries.tau(); std::cout<<"\n";
        }

std::cout<<"\n pending= "; pendingDeliveries.tau(); std::cout<<"\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";


        nodes.removeNode(lastNodeId);                // remove the node from the nodes list
std::cout<<"\n nodes= "; nodes.tau(); std::cout<<"\n";

    };
std::cout<<"\n deliveries pending remove the corresponing pickups:"<<pendingDeliveries.size();
//       while (not pendingDeliveries.size()==0) {
std::cout<<"\n\t removng:";
            pendingId =  pendingDeliveries.getnid(0);
            Order order = ordersList[ pendingDeliveries.getnode(0).getoid()  ];
order.debugdump();
            Dpnode node= order.getPickup();
            truck.removeOrder(  order );
            pendingDeliveries.removeNode ( pendingId );
            nodes.push_back(node);
            twc.recreateRowColumn( node.getnid() );
//       }

twc.dump();
std::cout<<"\n pending= "; pendingDeliveries.tau(); std::cout<<"\n";
    fleet.push_back(truck);
};
}



/*
    fleet.clear();
    Vehicle truck(depot,Q);
    Order order;
    Dpnode nodeToInsert,node;
    Vehicle nodes=originalnodes;
//OK    Vehicle pickups; 
    std::deque<Dpnode> used;
    Vehicle pendingDeliveries;   // delivery nodes not inserted yet 
    int lastNodeId,lastNodePos, bestPos,bestId,actualCompat,deliveryPos,pendingId;
//OK for (int i=0;i<originalnodes.size();i++) if (originalnodes.getnode(i).ispickup()) pickups.push_back(originalnodes.getnode(i));
//    while (nodes.hasNodes()) {   
std::cout<<"\n nodes= ";
nodes.tau();
       truck.clean();             //create a new truck for a route (depot has being inserted as first node)
       lastNodeId=depot.getnid(); 			//from all my depots/car relationship (in this case I only have one I find the best)
       lastNodePos=originalnodes.getpos(lastNodeId);
       maskVertical(lastNodePos);  		    //mask based on nodes id instead of position
       nodes.removeNode(lastNodeId);	            //already being used (before entering the while)
std::cout<<"\n starting route nodes= ";
nodes.tau();
int k=0;
       while ( nodes.hasNodes()) {   
           k++;
           maskVertical(lastNodePos);                //shrink the twc aka.. not considering going back to depot (has already being done????)

           bestPos=getBestPickupCompatible(lastNodePos);      //making sure I get a pickup first (is it a seed????)
           if (bestPos==0) break;                    //no compatible node, have to leave
           bestId=originalnodes.getnid(bestPos);     // find the nodes id      
           nodeToInsert=originalnodes.getnode(bestPos);
	   actualCompat=compat(lastNodePos,bestPos);
std::cout<<"\nbestPos= "<<bestPos;
std::cout<<"\tbestId= "<<bestId;
std::cout<<"\tactualCompat= "<<actualCompat;
           if (pendingDeliveries.hasNodes()) {
std::cout<<"\n\t deliveries pending:"<<pendingDeliveries.size();
                for (int j=0; j<pendingDeliveries.size();j++) {
                    deliveryPos=originalnodes.getpos( pendingDeliveries.getnid(j)); 
std::cout<<"\ndelivery Pos= "<<deliveryPos;
		    if ( actualCompat<compat(lastNodePos,deliveryPos) ) {
std::cout<<"better to insert the delivery";
                        nodeToInsert=pendingDeliveries.getnode(j) ;
                        bestId=pendingDeliveries.getnid(j);
                        bestPos=deliveryPos;
                        break;
                    };
                 }
           }
           //nodeToInsert=originalnodes.getnode(bestPos);
nodeToInsert.dump();
           truck.push_back(nodeToInsert);  //add the node to the truck
           if (nodeToInsert.ispickup()) {
std::cout<<"\tgetoId= "<<nodeToInsert.getoid();   //the order to which it belongs
                 pendingDeliveries.push_back (  ordersList[ nodeToInsert.getoid() ].getDelivery() );
           } else {
                 pendingDeliveries.removeNode(bestId);
           }

std::cout<<"\n pending= ";
pendingDeliveries.tau();

std::cout<<"\n truck= ";
truck.tau();
           nodes.removeNode(bestId);                // remove the node from the nodes list
std::cout<<"\n nodes= ";
nodes.tau();
           maskHorizontal(lastNodePos);              // lastInserted node is not reachable from any other node (aka, its already being used)
           lastNodeId=bestId;
           lastNodePos=bestPos;
           maskVertical(lastNodePos);                //shrink the twc aka.. not considering going back to depot (has already being done????)

	   // delaing with the delivery node  I inserted a pickup now its time to choose between a new pickup or the delivery
	   



       }
std::cout<<"\n deliveries pending remove the corresponing pickups:"<<pendingDeliveries.size();
//       while (not pendingDeliveries.size()==0) {
std::cout<<"\n\t removng:";
	    pendingId =  pendingDeliveries.getnid(0);
            order = ordersList[ pendingDeliveries.getnode(0).getoid()  ];
order.debugdump();
            node= order.getPickup();
            truck.removeOrder(  order );
            pendingDeliveries.removeNode ( pendingId );
            nodes.push_back(node);
            recreateRowColumn( node.getnid() );
//       }
            

std::cout<<"\n pending shoud be clean= ";
pendingDeliveries.tau();
       

       fleet.push_back(truck);
std::cout<<"\n final nodes= ";
nodes.tau();
//    }
*/



	  /* 
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
     }*/



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
          //here find a best order based on twcof depot of last order
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
