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

void Init_pd::insert(Vehicle &truck, int nodeId,Bucket &nodes) {
       Dpnode nodeToInsert=twc.getNode(nodeId);
       truck.push_back(nodeToInsert);  //add the node to the truck
       nodes.removeNode(nodeId);                // remove the node from the nodes bucket
       twc.maskVertical(nodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)
}

bool Init_pd::isCompatibleWithPending(int fromId,const Bucket &pending) {
       int currId;
       for (int i=0; i<pending.size(); i++) {
           currId=pending[i].getnid();
std::cout<<"compt "<<fromId<<" to "<< currId<<": "<<twc.compatibleIJ(fromId,currId)<<"\n";
           if (currId==fromId) continue;
           if (not twc.isCompatibleIJ(fromId,currId)) return false;
       }
       return true;
}


void Init_pd::removeIncompatible(int fromId,Bucket &nodes, Bucket &incompatible){
       int currId,pickId,deliverId;
       Order order;
       int i = 0;
       while (i<nodes.size()) {
           currId=nodes[i].getnid();
           if (currId==fromId) {i++;continue;};
           order =getOrderData(currId,pickId,deliverId);
//order.dump();std::cout<<"\n";
           if (currId==pickId) {
               if ( twc.isCompatibleIJ(fromId,pickId) and twc.isCompatibleIJ(fromId,deliverId)) i++;
               else { // either pick or delivery are incompatible
                   nodes.removeOrder(order);
                   incompatible.pushOrder(order);
                   i=0;
               }
           } else i++;
       }
}

void Init_pd::paperConst(Vehicle &truck, Bucket &nodes, Bucket &incompatible, Bucket &pending) {
        int lastNodeId;
        int insertedId,bestId,pickId,deliverId,pendingBestId;
std::cout<<"\n**RECURSION DATA******************************* \n";
std::cout<<"Entering nodes= "; nodes.tau(); std::cout<<"\n";
std::cout<<" truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" incompatible= "; incompatible.tau(); std::cout<<"\n";
std::cout<<" pending= "; pending.tau(); std::cout<<"\n";
std::cout<<"\n********************************* \n";
        if (nodes.isEmpty()) return; //no more compatible nodes
        lastNodeId = truck[truck.size()-1].getnid(); //last node in route

        if (truck.isEmptyTruck() ) bestId=twc.getSeed(lastNodeId,nodes);   //either its seed or its best
        else bestId=twc.getBestCompatible(lastNodeId,nodes);

        pendingBestId= twc.getBestCompatible(lastNodeId,pending);   //-1 when there is no pending?
std::cout<<"\nbestId= "<<bestId;
std::cout<<"\tpendingBestId= "<<pendingBestId<<"\n";
 
        if (bestId==-1) { std::cout<<"\nNever should I reach here??? "; return; };                    //no compatible node, have to leave //what with the other nodes???
        Order order =getOrderData(bestId,pickId,deliverId);
order.dump();std::cout<<"\n";

        if (inPending(pending,bestId)) { // delivery already in the route

            if (twc.isCompatibleIJ(lastNodeId, deliverId) ) {} //std::cout<<"\nse puede llegar al deliver desde lasNodeID";
            else std::cout<<"\nse puede llegar al pick desde lasNodeID TEORICMENTE ESTO NO SALE";


            if ( isCompatibleWithPending(bestId,pending) ){}  // std::cout<<"\nse puede llegar a todos los pendings desde el bestId  ";
            else std::cout<<"\nTEORICAMENTE ESTO NO SALE No se puede llegar a algun pending desde el pick  CUALLLLL???? ";
            


                pending.removeNode(bestId);
                insertedId=deliverId;
std::cout<<"\n1 delivery is already in the route ";
std::cout<<"\t insertedId= "<<insertedId<<"\n";

        } else {   

            //si es un deliver hay que meter primero el pick
            bestId=pickId;

            if (twc.isCompatibleIJ(lastNodeId, pickId) ) {} //std::cout<<"\nse puede llegar al pick desde lasNodeID";
            else std::cout<<"\nse puede llegar al pick desde lasNodeID TEORICMENTE ESTO NO SALE";

            if (twc.isCompatibleIJ(lastNodeId, deliverId) ){} // std::cout<<"\nse puede llegar al deliver desde lasNodeID";
            else std::cout<<"\nse puede llegar al pick desde lasNodeID TEORICMENTE ESTO NO SALE";

            
            if ( isCompatibleWithPending(pickId,pending) ) {} // std::cout<<"\nse puede llegar a todos los pendings desde el  pick ";
            else {
                 //std::cout<<"\nNo se puede llegar a algun pending desde el pick  CUALLLLL???? ";
                 //Hay que meter el pending 
                 bestId=pendingBestId;
                 pending.removeNode(bestId);
                 insertedId=bestId;
                 removeIncompatible(insertedId,nodes,incompatible); 
                 insert(truck,insertedId,nodes);
                 paperConst(truck,nodes,incompatible,pending);
                 return;
            }


            if ( isCompatibleWithPending(deliverId,pending) ) {} // std::cout<<"\nse puede llegar a todos los pendings desde el  deliver ";
            else std::cout<<"\nFALTA LA MANIPULACION No se puede llegar a algun pending desde el deliver  CUALLLLL???? ";


std::cout<<"\n2 bestId is a pickId ";
            if (not isCompatibleWithPending(pickId,pending)) {
                //inserto el mejor pending
                if (pendingBestId!=-1 and twc.isCompatibleIJ(pendingBestId,bestId) and twc.compatibleIJ(pendingBestId,bestId) > twc.compatibleIJ(bestId,pendingBestId) ) {
                    bestId=pendingBestId;
                    pending.removeNode(bestId);
                    insertedId=bestId;
std::cout<<"\n2.1 inserting bestPending";
std::cout<<"\t insertedId= "<<insertedId<<"\n";
                } else {
                   pending.push_back(order.getDelivery());
                   insertedId=pickId;
std::cout<<"\n2.2 inserting the pickup";
std::cout<<"\t insertedId= "<<insertedId<<"\n";
                }
            } else {
                if (pendingBestId!=-1 and twc.isCompatibleIJ(pendingBestId,bestId) and twc.compatibleIJ(pendingBestId,bestId) > twc.compatibleIJ(bestId,pendingBestId) ) {
                    bestId=pendingBestId;
                    pending.removeNode(bestId);
                    insertedId=bestId;
std::cout<<"\n2.1 inserting bestPending";
std::cout<<"\t insertedId= "<<insertedId<<"\n";
                } else {
                 pending.push_back(order.getDelivery());
                 insertedId=pickId;
std::cout<<"\n3.1 inserting the pickup";
std::cout<<"\t insertedId= "<<insertedId<<"\n";
               }
            }
         }
        //move incompatible from nodes bucket to incompatible bucket
//std::cout<<"\ninsertedId= "<<insertedId<<"\n";

       removeIncompatible(insertedId,nodes,incompatible); 
       insert(truck,insertedId,nodes);
     // quiza aqui revise is al borrar Incompatibles se borra un pending??? 
       paperConst(truck,nodes,incompatible,pending);
}    





// hanndling 2 things the nodes position in the original table and the nodes id 
void Init_pd::seqConst() {
std::cout << "Enter Problem::seqConst\n";
/* I already have the twc table ready */

    Bucket nodes;
    Bucket pendingDeliveries, incompatible;   // delivery nodes not inserted yet 
    nodes=originalnodes;
    int lastNodeId;

    Vehicle truck(depot,Q);

    for (int i=0;i<ordersList.size();i++)
        twc.setIncompatible(ordersList[i].getdid(),ordersList[i].getpid());
twc.dump();
int k=0;
    while (k<K and  nodes.hasNodes()) {   
std::cout<<"\n********************CYCLE=DATA****************************** "<<k<<"\n";
        truck.clean();             //create a new truck for a route (depot has being inserted as first node)
        lastNodeId=depot.getnid(); 			// I work with nids at this levle
        nodes.removeNode(lastNodeId);                // remove the node from the nodes list
        twc.recreateRowColumn( lastNodeId );
std::cout<<" nodes= "; nodes.tau(); std::cout<<"\n";
std::cout<<" incompatible= "; incompatible.tau(); std::cout<<"should be empty\n";
std::cout<<" pending= "; pendingDeliveries.tau(); std::cout<<"should be empty\n"; 
std::cout<<"\n********************CYCLE=ENDATA****************************** "<<k<<"\n";
        paperConst(truck,nodes,incompatible,pendingDeliveries);
std::cout<<"\n********************CYCLE=RESULTS****************************** "<<k<<"\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" nodes= "; nodes.tau(); std::cout<<"\n";
std::cout<<" incompatible= "; incompatible.tau(); std::cout<<"\n";
std::cout<<" pending= "; pendingDeliveries.tau(); std::cout<<"should be empty\n"; 
std::cout<<"\n********************CYCLE= END RESULTS****************************** "<<k<<"\n";
        nodes=incompatible;
        incompatible.erase();
        fleet.push_back(truck);
        k++;
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
