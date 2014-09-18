#include "init_pd.h"


void Init_pd::makeRoute(Vehicle &truck, Orders &orders, Orders &incompatible,Order lastOrder) {
std::cout << "Enter Make Route********************Recursion=DATA\n";
std::cout<<" truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders "; orders.dump(); 
std::cout<<" incompatible "; incompatible.dump(); 
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";
std::cout << "********************\n";
          if (orders.empty()) return; //no more compatible orders
          int bestAt;
          int pickPosFIFO,delPosFIFO;
          int pickPosLIFO,delPosLIFO;
          int pickPosPUSH,delPosPUSH;
          int pickPos,delPos;
          double pushCost,fifoCost,lifoCost;
          pickPosFIFO=delPosFIFO= pickPosLIFO=delPosLIFO= pickPosPUSH=delPosPUSH= pickPos=delPos=-1;
          pushCost=fifoCost=lifoCost=_MAX();
          Order order;
          if (truck.isEmptyTruck()) {
               bestAt=orders.leastReachable();
               order= orders[bestAt];
               orders.erase(bestAt);
               orders.removeIncompatible(order,incompatible);
               truck.pushOrder(order);
               lastOrder=order;
               makeRoute(truck,orders,incompatible,lastOrder);
               return;
          }
          //bestAt=orders.reachesMost();
          bestAt=orders.leastReachable();
          if (bestAt==-1) return; //no feasable best order
          order=orders[bestAt];
          orders.erase(bestAt);
order.dump();
std::cout<<"Retreiving costs \n";
          if (orders.isLIFOcompat(lastOrder,order)) lifoCost=truck.testInsertLIFO(order, orders, pickPosLIFO, delPosLIFO, twc);
          if(orders.isFIFOcompat(lastOrder,order)) fifoCost=truck.testInsertFIFO(order, orders, pickPosFIFO, delPosFIFO, twc);
          if (orders.isPUSHcompat(lastOrder,order)) pushCost=truck.testInsertPUSH(order, orders, pickPosPUSH, delPosPUSH, twc);
std::cout<<"LIFO Cost "<<lifoCost<<"\t pickPos "<<pickPosLIFO<<"\tdelpos "<<delPosLIFO<<"\n";
std::cout<<"FIFO Cost "<<fifoCost<<"\t pickPos "<<pickPosFIFO<<"\tdelpos "<<delPosFIFO<<"\n";
std::cout<<"PUSH Cost "<<pushCost<<"\t pickPos "<<pickPosPUSH<<"\tdelpos "<<delPosPUSH<<"\n";
          if      (lifoCost<_MAX() and lifoCost<=fifoCost and lifoCost<=pushCost) { pickPos=pickPosLIFO;delPos= delPosLIFO;}
          else if (fifoCost<_MAX() and fifoCost< lifoCost and fifoCost<=pushCost) { pickPos=pickPosFIFO;delPos= delPosFIFO;}
          else if (pushCost<_MAX() and pushCost< fifoCost and pushCost< lifoCost) { pickPos=pickPosPUSH;delPos= delPosPUSH;}
          else incompatible.push_back(order);

          if ( not (pickPos==-1 or delPos==-1) ) {
             truck.insertPOS(order,pickPos,delPos);
             orders.removeIncompatible(order,incompatible);
             lastOrder=order;
          }

          makeRoute(truck,orders,incompatible,lastOrder);
}







void Init_pd::initialConstruction() {
    std::cout << "Enter Problem::initialConstruction\n";
    Orders orders=ordersList;
    Orders incompatible;
orders.dump();
orders.dumpCompat();
twc.dump();
    int k=0;
    Order lastOrder;
    while (k<K and not orders.empty()) {
std::cout<<"\n********************CYCLE=DATA****************************** "<<k<<"\n";
        Vehicle truck(depot,Q);
std::cout<<" truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders "; orders.dump(); std::cout<<"\n";
std::cout<<" incompatible should be empty"; incompatible.dump();
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";

        makeRoute(truck,orders,incompatible,lastOrder);
        orders.join(incompatible);
        incompatible.clear();
        fleet.push_back(truck);
dump();
std::cout<<"\n********************CYCLE=RESULTS****************************** "<<k<<"\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders= "; orders.dump(); std::cout<<"\n";
std::cout<<" incompatible "; incompatible.dump(); std::cout<<"should be empty\n";
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";
std::cout<<"\n********************CYCLE= END RESULTS****************************** "<<k<<"\n";
        k++;
    }
std::cout<<"after 3\n";
    plot("InitialSolution","Initial Solution");
}



