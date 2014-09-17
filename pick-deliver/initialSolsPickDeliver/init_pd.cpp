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
          double pushCost,fifoCost,lifoCost=std::numeric_limits<double>::max();
          Order order;
          if (truck.isEmptyTruck()) {
               bestAt=orders.leastReachable();
               order= orders[bestAt];
               lastOrder=order;
               truck.pushOrder(order);
               orders.erase(bestAt);
               orders.removeIncompatible(order,incompatible);
               lastOrder=order;
               makeRoute(truck,orders,incompatible,lastOrder);
               return;
          }
          bestAt=orders.reachesMost();
          if (bestAt==-1) return; //no feasable best order
          order=orders[bestAt];
          orders.erase(bestAt);
order.dump();
std::cout<<"Retreiving costs \n";
          if (orders.isLIFOcompat(lastOrder,order)) {//lifo comes first
               lifoCost=truck.testInsertLIFO(order, orders, pickPosLIFO, delPosLIFO, twc);
std::cout<<"LIFO Cost "<<lifoCost<<"\t pickPos "<<pickPosLIFO<<"\tdelpos "<<delPosLIFO<<"\n";
               if (lifoCost<std::numeric_limits<double>::max()) {
                   truck.insertPOS(order,pickPosLIFO,delPosLIFO);
                   orders.removeIncompatible(order,incompatible);
                   lastOrder=order;
                   makeRoute(truck,orders,incompatible,lastOrder);
                   return;
               };
          }  else std::cout<<"Not LIFO\n";
          if(orders.isFIFOcompat(lastOrder,order)) {
               fifoCost=truck.testInsertFIFO(order, orders, pickPosFIFO, delPosFIFO, twc);
               if (fifoCost<std::numeric_limits<double>::max()) {
std::cout<<"FIFO Cost "<<fifoCost<<"\t pickPos "<<pickPosFIFO<<"\tdelpos "<<delPosFIFO<<"\n";
                   truck.insertPOS(order,pickPosFIFO,delPosFIFO);
                   orders.removeIncompatible(order,incompatible);
                   lastOrder=order;
                   makeRoute(truck,orders,incompatible,lastOrder);
                   return;
               }
          }  else std::cout<<"Not FIFO\n";
          if (orders.isPUSHcompat(lastOrder,order)) {
               pushCost=truck.testInsertPUSH(order, orders, pickPosPUSH, delPosPUSH, twc);
               if (pushCost<std::numeric_limits<double>::max()) {
std::cout<<"PUSH Cost "<<pushCost<<"\t pickPos "<<pickPosPUSH<<"\tdelpos "<<delPosPUSH<<"\n";
                   truck.insertPOS(order,pickPosPUSH,delPosPUSH);
                   orders.removeIncompatible(order,incompatible);
                   lastOrder=order;
                   makeRoute(truck,orders,incompatible,lastOrder);
                   return;
               }
          }

std::cout<<"unfeasable so its incompatible \n";
         incompatible.push_back(order);
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



