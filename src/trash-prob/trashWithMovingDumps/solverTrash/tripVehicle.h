#ifndef TRIPVEHICLE_H
#define TRIPVEHICLE_H

#include "vehicle.h"


// TODO move the code to .cpp keep the headers

class Trip: public Vehicle1 {

};


class Vehicle: public Vehicle1 {
 public:


  Vehicle():Vehicle1(){};
  Vehicle(const std::string &line, const Bucket &otherlocs )
    : Vehicle1(line, otherlocs)   { };
  Vehicle( int _vid, int _start_id, int _dump_id, int _end_id,
           int _capacity, int _dumpservicetime, int _starttime,
           int _endtime, const Bucket &otherlocs )
    : Vehicle1( _vid, _start_id, _dump_id, _end_id,
                   _capacity, _dumpservicetime, _starttime,
                   _endtime, otherlocs ) {};


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
#endif // TRIPVEHICLE_H
