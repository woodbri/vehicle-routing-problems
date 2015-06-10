#ifndef FLEET_H_
#define FLEET_H_

#include "tripVehicle.h"

class Fleetopt: public Vehicle {
  std::deque< Vehicle > fleet;
  
  public:
  // default constructor & destructor
  std::deque<Vehicle> get_opt_fleet();
  void optimize();
  void insert(const std::deque < Vehicle> &p_fleet);

  void extract_trips();
};

#endif // "FLEET_H_"

