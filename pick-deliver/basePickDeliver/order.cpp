#include "dpnode.h"
#include "order.h"


bool Order::checkIntegrity(const int ordersCant) const {
     bool flag=true;
     if (oid<0 or oid>ordersCant) {
        std::cout << "Order["<<oid<<"]: Order oid out of range:"<<oid<<"expected in [0,"<<ordersCant<<"] \n";
        flag=false;}
     return flag;
}



void Order::debugdump() const {
    std::cout << "Order#"<<oid << ":  "
              << getpid() << ", "
              << getdid() << ", ";
std::cout              << getdistPickupDepot() <<  ", "
              << getdistDeliveryDepot() << std::endl;
    std::cout <<  "Pickup \n";
    pickup->dump();
    std::cout <<  "Delivery \n";
    delivery->dump();
}


void Order::dump() const {
    if (oid!=-1) std::cout << "(" << getpid() << "," << getdid() << ") "; 
    else  std::cout << "(-1,-1)";
}
