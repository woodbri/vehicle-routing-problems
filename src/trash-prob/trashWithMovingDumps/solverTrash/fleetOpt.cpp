#include "fleetOpt.h"

  
void Fleetopt::insert(const std::deque <Vehicle> &p_fleet) {
  fleet = p_fleet;
}

void Fleetopt::extract_trips() {
  for (const auto  &truck: fleet) {
    for (const auto  &trip: truck.trips) {
      trips.push_back(trip);
    }
  }
  tauTrips();
}

void Fleetopt::optimize() {
  extract_trips();
}


std::deque<Vehicle> Fleetopt::get_opt_fleet() {
  return fleet;
}

