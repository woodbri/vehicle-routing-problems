#include "vehicle.h"


// TODO move the code to .cpp keep the headers

class Trip: public Vehicle {

};


class Truck: public Vehicle {
 public:

  std::deque< Trip > trips;


  void add_trip(const Trip &p_trip) {

    Trip trip = p_trip;
    tau("before inserting trip");
    this->e_add_trip(trip);
    tau("after inserting trip");
    
    // the start of the trip does not change
    // the dump of the trip must have the same values as the dump of the truck
    trip.getCostOsrm();
    getCostOsrm();
    trip.print_short_eval();
    print_short_eval();

    assert(true==false);
    
    
  }
};

