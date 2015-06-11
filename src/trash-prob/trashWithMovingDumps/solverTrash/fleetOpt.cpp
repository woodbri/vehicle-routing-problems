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
}

void Fleetopt::optimize() {
  extract_trips();
  tauTrips("fleet::AFTER extract");

  intraTripOptimizationNoOsrm();
  auto count = 0;
  auto tot_count = 0;

  //tauTrips("fleet::AFTER intraTripOptimizationNoOsrm");

  for (UINT i = 0; i < 2; ++i) {
    count = exchangesWorse(10);
    tot_count += count;
    //DLOG(INFO) << "fleet::exchangeAllWorse Performed: " << count;
  }
  //DLOG(INFO) << "fleet::total exchangeAllWorse Performed: " << count;
  //tauTrips("fleet::AFTER exchange all worse");
  int n = 10;
  for (UINT i = 0; i < n; ++i) {
    for (auto &trip : trips) {
      for (auto &o_trip : trips) {
        DLOG(INFO) << "exchanges " << trip.getVid() << "," << trip.trip_id() << " with " << o_trip.getVid() << " , " << o_trip.trip_id();
        exchangesWithOnPath(trip, o_trip);
      }
    }
  }
//  tauTrips("fleet::AFTER N cycles of exchanges");
  intraTripOptimizationNoOsrm();
  tauTrips("fleet::AFTER intraTripOptimizationNoOsrm");

}


std::deque<Vehicle> Fleetopt::get_opt_fleet() {
  return fleet;
}

