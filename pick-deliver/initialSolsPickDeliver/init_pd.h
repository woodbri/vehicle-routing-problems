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
#include "twc.h"

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

