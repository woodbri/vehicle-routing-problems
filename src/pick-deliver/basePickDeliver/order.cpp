/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
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
