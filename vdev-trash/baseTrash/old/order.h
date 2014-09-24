#ifndef ORDER_H
#define ORDER_H

#include <iostream>
#include "dpnode.h"

class Order {
private:
   
  public:
    int oid;        // order id
    double dist;    // distance from depot to pickup location  can be calculated
    double dist2;   // distance from delivery to depot
    Dpnode *pickup;
    Dpnode *delivery;



Order(Dpnode &p,Dpnode &d , int i_oid, const Dpnode depot){ 
      fillOrder(p,d,i_oid,depot);
}

void fillOrder(Dpnode &p, Dpnode &d , int i_oid, const Dpnode &depot){ 
      oid=i_oid;
      pickup=&p;
      delivery=&d;
      pickup->setoid(i_oid);  //tell pickup and delivery they belong to the order
      delivery->setoid(i_oid);
      dist=pickup->distance(depot);
      dist2=delivery->distance(depot);
}



Order(const Order& other){
      oid=other.oid;
      pickup=other.pickup;
      delivery=other.delivery;
};

Order(){ 
         oid=-1;
         dist=dist2=0;
         pickup=NULL;
         delivery=NULL;
}

/*************accesosrs*/
int getpid() const {return oid==-1?oid:pickup->getnid();}
int getdid() const {return oid==-1?oid:delivery->getnid();}
int getoid() const {return oid;};
Dpnode getDelivery() const {Dpnode node; node = *delivery; return node; };
Dpnode getPickup() const {Dpnode node; node = *pickup; return node; };

double getdistPickupDepot() const {return dist;};
double getdistDeliveryDepot() const {return dist2;};
double getdistPickupOther(const Dpnode other) const {return pickup->distance(other);};
double getdistDeliveryOther(const Dpnode other) const {return pickup->distance(other);};
double getdistPickDeliver() const {return pickup->distance(*delivery);};
/************state*/
bool checkIntegrity(const int nodesCant) const;
/************ mutators*/
void  moveOrder(const int toRoute);
/************ output */
void debugdump() const;
void dump() const;
};

#endif
