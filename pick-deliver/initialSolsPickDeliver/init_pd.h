#ifndef INIT_PD_H
#define INIT_PD_H

#include <deque>
#include <cmath>

#include "order.h"
#include "orders.h"
#include "prob_pd.h"
#include "solution.h"
#include "vehicle.h"
#include "bucketn.h"
#include "plot.h"

//const double EPSILON = 0.001;

class Init_pd: public Solution {

  
  public:
    Init_pd(const Solution& P):Solution(P){
         initialConstruction();
    }
    void makeRoute(Vehicle &truck, Orders &orders, Orders &incompat,Order lastOrder);
    void initialConstruction();
    void e_performInsertion(Vehicle &truck,const Order &order,int pickPos,int delPos,Orders &orders,Orders &incompat);

};
#endif

