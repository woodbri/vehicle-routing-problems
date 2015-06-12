#include "fleetOpt.h"

void Optimizer::optimizefleet(int iter) {
    Fleetopt opt_fleet;
    opt_fleet.insert(fleet);
    opt_fleet.optimize(iter);
    fleet.clear();
    fleet = opt_fleet.get_opt_fleet();
    DLOG(INFO) << "FINAL OPTIMIZATION\n";
    tau();
}


  
void Fleetopt::insert(const std::deque <Vehicle> &p_fleet) {
  fleet = p_fleet;
}

void Fleetopt::extract_trips() {
  for (const auto  &truck: fleet) {
    for (const auto  &trip: truck.trips) {
      trips.push_back(trip);
    }
  }
}

void Fleetopt::optimize(int iter) {
  extract_trips();
  tauTrips("fleet::AFTER extract");

  intraTripOptimizationNoOsrm();
  auto count = 0;
  auto tot_count = 0;

  tauTrips("fleet::AFTER intraTripOptimizationNoOsrm");

  for (UINT i = 0; i < 2; ++i) {
    count = exchangesWorse(10);
    tot_count += count;
    //DLOG(INFO) << "fleet::exchangeAllWorse Performed: " << count;
  }
  DLOG(INFO) << "fleet::total exchangeAllWorse Performed: " << count;
  tauTrips("fleet::AFTER exchange all worse");
  count = 0;
  tot_count = 0;
  for (UINT i = 0; i < iter; ++i) {
    for (auto &trip : trips) {
      for (auto &o_trip : trips) {
        count +=exchangesWithOnPath(o_trip, trip);
        count +=exchangesWithOnPath(trip, o_trip);
      }
    }
  }
  intraTripOptimizationNoOsrm();
  tauTrips("fleet::AFTER intraTripOptimizationNoOsrm");

}


std::deque<Vehicle> Fleetopt::get_opt_fleet() {
  return fleet;
}

