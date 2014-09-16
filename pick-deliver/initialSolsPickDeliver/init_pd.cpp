#include "init_pd.h"


void Init_pd::makeRoute(Vehicle &truck, Orders &orders, Orders &incompatible,Order lastOrder) {
std::cout << "Enter Make Route********************Recursion=DATA\n";
std::cout<<" truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders "; orders.dump(); 
std::cout<<"\n incompatible "; incompatible.dump(); 
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";
          if (orders.empty()) return; //no more compatible orders
          int bestAt,pickPos,delPos;
          int lastPickInsertedAt,lastDeliveryInsertedAt;
          double pushCost;
          Order order;
          if (truck.isEmptyTruck()) {
               bestAt=orders.leastReachable();
               order= orders[bestAt];
               lastOrder=order;
               truck.pushOrder(order);
               orders.erase(bestAt);
               orders.removeIncompatible(order,incompatible);
          } else {
               bestAt=orders.reachesMost();
               if (bestAt==-1) return; //no feasable best order
               order=orders[bestAt];
               orders.erase(bestAt);
               if (orders.isPUSHcompat(lastOrder,order)) {
                   pushCost=truck.testInsertPUSH(order, orders, pickPos, delPos, twc);
std::cout<<"PUSH Cost "<<pushCost<<"\t pickPos "<<pickPos<<"\tdelpos "<<delPos<<"\n";
                   truck.pushOrder(order);
                   orders.removeIncompatible(order,incompatible);
                   lastOrder=order;
               } else  {
                    incompatible.push_back(order);   //cant pushback
               }
         } 
std::cout<<"\n********************Recursion=RESULT\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders "; orders.dump(); std::cout<<"\n";
std::cout<<" incompatible "; incompatible.dump(); std::cout<<"\n";
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";
std::cout<<"\n********************recursion= END RESULTS****************************** \n";
         makeRoute(truck,orders,incompatible,lastOrder);
}



void Init_pd::initialConstruction() {
    std::cout << "Enter Problem::initialConstruction\n";
    Orders orders=ordersList;
    Orders incompatible;
orders.dump();
orders.dumpCompat();
twc.dump();
    Vehicle truck(depot,Q);
    int k=0;
    Order lastOrder;
    while (k<K and not orders.empty()) {
std::cout<<"\n********************CYCLE=DATA****************************** "<<k<<"\n";
        truck.e_clean(); //create a new truck for a route (depot has being inserted as first node)
std::cout<<" truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders "; orders.dump(); std::cout<<"\n";
std::cout<<" incompatible should be empty"; incompatible.dump();
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";

        makeRoute(truck,orders,incompatible,lastOrder);

std::cout<<"\n********************CYCLE=RESULTS****************************** "<<k<<"\n";
std::cout<<"\n truck= "; truck.tau(); std::cout<<"\n";
std::cout<<" orders= "; orders.dump(); std::cout<<"\n";
std::cout<<" incompatible "; incompatible.dump(); std::cout<<"should be empty\n";
std::cout<<" lastOrder ";lastOrder.dump(); std::cout<<"\n";
std::cout<<"\n********************CYCLE= END RESULTS****************************** "<<k<<"\n";

        orders.join(incompatible);
std::cout<<"before 1\n";
        incompatible.clear();
std::cout<<"before 2\n";
        fleet.push_back(truck);
std::cout<<"before 3\n";
        k++;
    }
std::cout<<"after 3\n";
}



