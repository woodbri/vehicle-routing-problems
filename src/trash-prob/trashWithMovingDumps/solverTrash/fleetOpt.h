#ifndef FLEET_H_
#define FLEET_H_

#include "solution.h"
#include "tripVehicle.h"

class Fleetopt: public Vehicle {
  std::deque< Vehicle > fleet;
  
  public:
  // default constructor & destructor
  std::deque<Vehicle> get_opt_fleet();
  void optimize(int iter);
  void insert(const std::deque < Vehicle> &p_fleet);

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

