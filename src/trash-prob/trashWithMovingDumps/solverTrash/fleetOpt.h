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
#ifndef FLEET_H_
#define FLEET_H_

#include "solution.h"
#include "tripVehicle.h"

class Fleetopt: public Vehicle {
  std::deque< Vehicle > fleet;
  std::deque< UINT > ids;
  
  public:
  // default constructor & destructor
  std::deque<Vehicle> get_opt_fleet();
  void optimize(int iter);
  void insert(const std::deque < Vehicle> &p_fleet);
  void reconstruct_fleet();

  void extract_trips();
};


class Optimizer: public Solution {
  public:
    void optimizefleet(int iter);
    Optimizer(const Solution &solution, int iter): Solution(solution) {
        optimizefleet(iter);
    }
};



#endif // "FLEET_H_"

