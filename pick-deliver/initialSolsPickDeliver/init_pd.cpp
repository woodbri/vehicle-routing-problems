#include <cmath>
#include <deque>
#include "plot1.h"
#include "vehicle.h"
#include "compatible.h"
#include "init_pd.h"

/* prerequisites 
 nodes sorted is based on amount of twc
 orders were made based on TWC 

*/


bool Init_pd::incompatible(int fromId, int toId) const   { return not twc.isCompatibleIJ(fromId,toId); }
/* examle for 2nd condition 
distanceTime between Py and Dx is 500

Py opens at 900 , closes at 1000 service time is 100 

Dx opens at 900 , closes at 1000 service time is 100 

so, 

if I arrive to Py at 900, I will arrive at Dx at 900+100+500=1500  which is way over the closing time of Dx

Therefore going from Py -> Dx is impossible

likewise, if I arrive to Dx at 900, I will arrive at Py at 900+100+500=1500  which is way over the closing time of Py

herefore going from Dx -> Py is impossible

*/
bool Init_pd::isIncompatibleOrder(const Order &orderx, const Order &ordery) const {
    int Px,Dx;
    int Py,Dy;
    Px= orderx.getpid();
    Dx= orderx.getdid();
    Py= ordery.getpid();
    Dy= ordery.getdid();
    if (  incompatible(Px,Py)  or incompatible(Px,Dy) ) return true;
    if (  incompatible(Py,Dx) and incompatible(Dx,Py) ) return true;
    if (  incompatible(Dy,Dx) and incompatible(Dx,Dy) ) return true;
    return false;
}

bool Init_pd::isIncompatibleOrder(int oidx,int oidy ) const  {
    Order orderx=ordersList[ oidx ];
    Order ordery=ordersList[ oidy ];
    return isIncompatibleOrder(orderx,ordery);
}

bool Init_pd::isCompatibleOrder(int oidx,int oidy) const { 
     return not isIncompatibleOrder(oidx,oidy); 
}

bool Init_pd::isCompatibleOrder(const Order &from, const Order &to) const  {
    return not isIncompatibleOrder(from.getoid(),to.getoid());
}

double Init_pd::compatibleIJ(int fromNid, int toNid) const {
      double value=twc.compatibleIJ(fromNid,toNid);
      return value== -std::numeric_limits<double>::max()? 0:value;
}

double Init_pd::compatibleIJ(const Order &order ) const {
    int Px,Dx;
    Px= order.getpid();
    Dx= order.getdid();
    return compatibleIJ(Px,Dx);
}


double Init_pd::compatOrdersMeasure(const Order &orderx, const Order &ordery) const {
    int Px,Dx;
    int Py,Dy;
    double total;
    Px= orderx.getpid();
    Dx= orderx.getdid();
    Py= ordery.getpid();
    Dy= ordery.getdid();
    if (isIncompatibleOrder(orderx,ordery)) return  -std::numeric_limits<double>::max();
/*
std::cout<<"\ncompat  =";
orderx.dump();
ordery.dump();
std::cout<<"\ncompat Px->Dx ="<<twc.compatibleIJ(Px,Dx);
std::cout<<"\ncompat Px->Py ="<<twc.compatibleIJ(Px,Py);
std::cout<<"\ncompat Px->Dy ="<<twc.compatibleIJ(Px,Dy);

std::cout<<"\ncompat Dx->Py ="<<twc.compatibleIJ(Dx,Py);
std::cout<<"\ncompat Dx->Dy ="<<twc.compatibleIJ(Dx,Dy);

std::cout<<"\ncompat Py->Dx ="<<twc.compatibleIJ(Py,Dx);
std::cout<<"\ncompat Py->Dy ="<<twc.compatibleIJ(Py,Dy);

std::cout<<"\ncompat Dy->Dx ="<<twc.compatibleIJ(Dy,Dx);
*/
total=   compatibleIJ(Px,Dx)+compatibleIJ(Px,Py)+compatibleIJ(Px,Dy)+
         compatibleIJ(Dx,Py)+compatibleIJ(Dx,Dy)+
         compatibleIJ(Py,Dx)+compatibleIJ(Py,Dy)+
         compatibleIJ(Dy,Dx);
//std::cout<<"\nTotal ="<<total<<"\n";

     return total;
}






void Init_pd::sortOrdersByIncompat(std::deque<Order> orders) {
    int j;
    Order tmp;
    for (int i=1; i<orders.size();i++) {
      tmp=orders[i];
//ordersdump(orders);
//std::cout<<"orders sort 1---"<<i<<"\n";
      //for (j = i; j > 0 and compatOrdersMeasure(orders[j-1],orders[j])> compatOrdersMeasure(orders[j],orders[j-1]);j-- ){
      for (j = i; j > 0;j-- ){
//std::cout<<"J="<<j<<"\n";
//std::cout<<"orders sort 2---   "<<compatOrdersMeasure(orders[j-1],tmp)<<" vs " <<compatOrdersMeasure(tmp,orders[j-1])<<" si ir de J-1 a j es mejor que ir de j a j-1 hace el cambio\n";
//ordersdump(orders);
//std::cout<<"orders are compatible???"<<isCompatibleOrder(orders[j-1],tmp);
//std::cout<<"reverse orders are compatible???"<<isCompatibleOrder(tmp,orders[j-1]);
         if ( compatOrdersMeasure(orders[j-1],tmp)<= compatOrdersMeasure(tmp,orders[j-1])){
         orders[j]=orders[j-1];
//std::cout<<"SWAPING orders sort 1----("<<i<<","<<j<<")\n";
//ordersdump(orders);
         } else break;
//ordersdump(orders);
      }
      orders[j]=tmp;
    }
//ordersdump(orders);
};





int Init_pd::countCompatibleOrders(const Order &from, const std::deque<Order> &to) const{
    if (to.empty()) return 0;
    int count=0;
    for (int i=0;i<to.size();i++) {
        if (from.getoid()==to[i].getoid()) continue;  //ignoring the same order
        if (isCompatibleOrder(from,to[i])) count++;
    }
    return count;
}

int Init_pd::getMostCompatibleOrder(const std::deque<Order> &orders) { 
    int bestAt=0;
    int bestCount=0;
    int currCount;
    if (orders.size()==0) return -1;
    for (int i=0;i<orders.size();i++) {
        currCount=countCompatibleOrders(orders[i],orders);
        if (currCount>bestCount) {
           bestAt=i;
           bestCount=currCount;
        };
    };
    return bestAt;
};

void Init_pd::removeIncompatibleOrders(const Order &from,  std::deque<Order> &orders, std::deque<Order> &incompatible) {
    for (int i=0;i<orders.size();i++) {
        if (isIncompatibleOrder(from,orders[i])) {
              incompatible.push_back(orders[i]);
              orders.erase(orders.begin()+i);
              i--;
        };
    }
}

void Init_pd::insertInTruck(Vehicle &truck,const Order &order){
    int deliverPosLR=0;
    int pickPosLR=0;
    int deliverPosRL=truck.size()-1;
    int pickPosRL=truck.size()-1;
    int Px= order.getpid();
    int Dx= order.getdid();

    for  (int i=0; i<truck.size();i++) {
        pickPosLR=i;
        if ( twc.isCompatibleIJ(truck[i].getnid(),Px) ) continue;
        else {break; };
    };

    for (int i=truck.size()-1; i>=0;i--) {
        pickPosRL=i;
        if ( twc.isCompatibleIJ(Px,truck[i].getnid()) ) continue;
        else {break; };
    };
    if (pickPosRL>pickPosLR) std::cout<<" SOMETHING WENT WRONG 1 \n";
    truck.insert(order.getPickup(),pickPosRL+1);

std::cout<<"\npick Pos LR="<<pickPosLR<<"\tpick Pos RL="<<deliverPosRL<<"\n";
truck.tau(); std::cout<<"\n";
    for  (int i=0; i<truck.size();i++) {
        deliverPosLR=i;
        if ( twc.isCompatibleIJ(truck[i].getnid(),Dx) ) continue;
        else {break;};
    };
    for (int i=truck.size()-1; i>=0;i--) {
        deliverPosRL=i;
        if ( twc.isCompatibleIJ(Dx,truck[i].getnid()) ) continue;
        else {break; };
    };
    // teoricamente Rl<LR
    if (deliverPosLR<deliverPosRL) std::cout<<" SOMETHING WENT WRONG 2";
    truck.insert(order.getDelivery(),deliverPosRL+1);
std::cout<<"\n deliver Pos LR="<<deliverPosLR<<"\tdeliver Pos RL="<<deliverPosRL<<"\n";
truck.tau(); std::cout<<"\n";
};

int Init_pd::getBestOrder(Vehicle &truck,const std::deque<Order> &orders)const {
    double bestCost= std::numeric_limits<double>::max();
    int bestAt=-1;
    Order bestOrder;
    for (int i=0;i<orders.size();i++) {
        if (truck.insertOrderAfterLastPickup(orders[i],twc)) {//was feasable also
             if (truck.getcost() < bestCost) {
                  bestAt=i;
                  bestOrder=orders[i];
             }
        }  //else was not feasable 
        truck.removeOrder(orders[i]); //remove to try next
    }
    return bestAt;
}
        



void Init_pd::makeRoute(Vehicle &truck, std::deque<Order> &orders, std::deque<Order> &incompatible) {
std::cout << "Enter Problem::orderConstraintConstruction\n";
std::cout<<"\n********************Recursion=DATA\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders= "; ordersdump(orders); std::cout<<"\n";
std::cout<<" incompatible= "; ordersdump(incompatible); std::cout<<"\n";
std::cout<<"\n********************recursion= END RESULTS****************************** \n";
    if (orders.empty()) return; //no more compatible orders
    int bestAt;
    int lastPickInserted;
    Order order;
    if (truck.isEmptyTruck())  { 
          order= orders[0];
          truck.pushOrder(order);
          orders.erase(orders.begin());
          removeIncompatibleOrders(order,orders,incompatible);
    } else {
          bestAt=getBestOrder(truck,orders);
          if (bestAt==-1) return;  //no feasable best order
          order=orders[bestAt];
                  
ordersdump(orders);
std::cout<<"\nbestSt"<<bestAt<<" \n ";
ordersdump(orders);
truck.tau();
          truck.insertOrderAfterLastPickup(order,twc);
std::cout<<"\nbestSt"<<bestAt<<" \n ";
ordersdump(orders);
truck.tau();
std::cout<<"\ngoing to rease\n ";
          orders.erase(orders.begin()+bestAt);
std::cout<<"\nafter erase\n ";
std::cout<<"\ngoing to remove\n ";
          removeIncompatibleOrders(order,orders,incompatible);
std::cout<<"\nafter remove\n ";
    }
std::cout<<"\n********************Recursion=RESULT\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders= "; ordersdump(orders); std::cout<<"\n";
std::cout<<" incompatible= "; ordersdump(incompatible); std::cout<<"\n";
std::cout<<"\n********************recursion= END RESULTS****************************** \n";
    makeRoute(truck,orders,incompatible);
}

    
void Init_pd::orderConstraintConstruction() {
std::cout << "Enter Problem::orderConstraintConstruction\n";
/* I already have the twc table ready */

    //BucketN nodes;
    //BucketN pendingDeliveries, incompatible;   // delivery nodes not inserted yet 
    std::deque<Order> orders=ordersList;
    std::deque<Order> incompatible;
    //nodes=originalnodes;
    //int lastNodeId;

    sortOrdersByIncompat(orders);
    ordersdump(orders);
    Vehicle truck(depot,Q);

    for (int i=0;i<ordersList.size();i++)
        twc.setIncompatible(ordersList[i]);
    
twc.dump();
int k=0;
    while (k<K and  not orders.empty()) {
std::cout<<"\n********************CYCLE=DATA****************************** "<<k<<"\n";
        truck.clean();             //create a new truck for a route (depot has being inserted as first node)
        //lastNodeId=depot.getnid();                      // I work with nids at this levle
        //nodes.removeNode(lastNodeId);                // remove the node from the nodes list
        //twc.recreateRowColumn( lastNodeId );
std::cout<<" orders= "; ordersdump(orders); std::cout<<"\n";
std::cout<<" incompatible= "; ordersdump(incompatible); std::cout<<"should be empty\n";
//std::cout<<" pending= "; pendingDeliveries.tau(); std::cout<<"should be empty\n";
std::cout<<"\n********************CYCLE=ENDATA****************************** "<<k<<"\n";
        makeRoute(truck,orders,incompatible);
std::cout<<"\n********************CYCLE=RESULTS****************************** "<<k<<"\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders= "; ordersdump(orders); std::cout<<"\n";
std::cout<<" incompatible= "; ordersdump(incompatible); std::cout<<"should be empty\n";
std::cout<<"\n********************CYCLE= END RESULTS****************************** "<<k<<"\n";
        orders.insert (orders.end(),incompatible.begin(),incompatible.end());
        incompatible.clear();
        fleet.push_back(truck);
        k++;
     }
}





/*        another solution                                      */
Order Init_pd::getOrderData(int nodeId, int &pick, int &deliver){
    Order order;
    Dpnode node=twc.getNode(nodeId);
    order=  ordersList[ node.getoid() ];
    pick= order.getPickup().getnid();
    deliver= order.getDelivery().getnid();
    return order;
}

bool Init_pd::inPending(const BucketN &pend, int nodeId) {
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

void Init_pd::insert(Vehicle &truck, int nodeId,BucketN &nodes) {
       Dpnode nodeToInsert=twc.getNode(nodeId);
       truck.push_back(nodeToInsert);  //add the node to the truck
       nodes.removeNode(nodeId);                // remove the node from the nodes bucket
       twc.maskVertical(nodeId);              // lastInserted node is not reachable from any other node (aka, its already being used)
}

bool Init_pd::isCompatibleWithPending(int fromId,const BucketN &pending) {
       int currId;
       for (int i=0; i<pending.size(); i++) {
           currId=pending[i].getnid();
std::cout<<"compt "<<fromId<<" to "<< currId<<": "<<twc.compatibleIJ(fromId,currId)<<"\n";
           if (currId==fromId) continue;
           if (not twc.isCompatibleIJ(fromId,currId)) return false;
       }
       return true;
}


void Init_pd::removeIncompatible(int fromId,BucketN &nodes, BucketN &incompatible){
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

void Init_pd::paperConst(Vehicle &truck, BucketN &nodes, BucketN &incompatible, BucketN &pending) {
        int lastNodeId;
        int insertedId,bestId,pickId,deliverId,pendingBestId;
std::cout<<"\n**RECURSION DATA******************************* \n";
std::cout<<"Entering nodes= "; nodes.tau(); std::cout<<"\n";
std::cout<<" truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" incompatible= "; incompatible.tau(); std::cout<<"\n";
std::cout<<" pending= "; pending.tau(); std::cout<<"\n";
std::cout<<"\n********************************* \n";
        if (nodes.empty()) return; //no more compatible nodes
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

    BucketN nodes;
    BucketN pendingDeliveries, incompatible;   // delivery nodes not inserted yet 
    nodes=originalnodes;
    int lastNodeId;

    Vehicle truck(depot,Q);

    for (int i=0;i<ordersList.size();i++)
        twc.setIncompatible(ordersList[i]);
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
truck.Path().e_movereverse(4,5,2,100);
        fleet.push_back(truck);
truck.Path().e_movereverse(0,4,3,100);
        fleet.push_back(truck);
truck.Path().e_movereverse(1,3,5,100);
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
    std::deque<Order> clientBucketN;
    std::deque<Order> waitOrders;
    sortOrdersbyDistReverse();
    clientBucketN=ordersList;
    while (!clientBucketN.empty()) {        //are there any unrouted customers
       Vehicle route(depot,Q);             // initialize tour  
       while (!clientBucketN.empty()) {     //are there any unrouted customers?
          order=clientBucketN.front();          
          clientBucketN.pop_front();
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
       clientBucketN=waitOrders;
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


