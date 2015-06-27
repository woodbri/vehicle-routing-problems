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

#include "pg_trashcheck.h"

#ifdef DOVRPLOG
#include "logger.h"
#endif


Trash_check::Trash_check(container_t *p_containers, unsigned int container_count,
                       otherloc_t *p_otherlocs, unsigned int otherloc_count,
                       ttime_t *p_ttimes, unsigned int ttime_count,
                       vehicle_t *p_vehicles, unsigned int vehicle_count)
{

  Bucket nodes;
  Bucket intersection;
#ifdef VRPMINTRACE
  DLOG(INFO) << "trashProb Check Problem -------------from sql data---------";
#endif

#if 0
  twc->emptiedTruck = false;
#endif 

  {  // checking the containers
     bool errorFound(CheckContainers(p_containers, container_count));
     if (errorFound || invalid.size() || (not pickups.size())) {
#ifdef VRPMINTRACE
        DLOG(INFO) << getErrorsString();
#endif
        return;
     }
  }

  {  // checking the other locations
     bool errorFound(CheckOtherlocs(p_otherlocs, otherloc_count));
     if (errorFound || invalid.size() || (not otherlocs.size())) {
#ifdef VRPMINTRACE
        DLOG(INFO) << getErrorsString();
#endif
        return;
     }
  }

  intersection = otherlocs * pickups;

  std::string errorStr;
  { // checking: the insersection must be empty
    if (intersection.size()) {
      for (UINT i = 0; i < intersection.size(); i++) {
        errorStr = "Container #" + numbertoString(intersection[i].id()) +
                   ": has same id as an other location's id";
        errorsFound.push_back(errorStr);
      };
#ifdef VRPMINTRACE
      DLOG(INFO) << getErrorsString();
#endif
      return;
    };

  }

  nodes = pickups + otherlocs;

  { // checking the vehicles
     bool errorFound(CheckVehicles(p_vehicles, vehicle_count));
     if (errorFound || invalidTrucks.size() || (not trucks.size())) {
#ifdef VRPMINTRACE
        DLOG(INFO) << getErrorsString();
#endif
        return;
     }
  }

  assert( trucks.size() and depots.size() and dumps.size() and endings.size() );



#ifdef VRPMINTRACE
  nodes.dump( "nodes" );
  dumps.dump( "dumps" );
  depots.dump( "depots" );
  pickups.dump( "pickups" );
  endings.dump( "endings" );
  datanodes.dump( "datanodes" );
#endif
}

static bool isLatLon(double x, double y)
{
  return       -180.0 <= x and x <= 180.0
               and  - 180.0 <= y and y <= 180.0;
}

bool Trash_check::CheckTimes(ttime_t *p_ttimes, int count) {
  for ( int i = 0; i < count; ++i ) {
#ifdef VRPMINTRACE
    DLOG( INFO ) << "checking pending";
#endif
  }
}

bool Trash_check::CheckContainers( container_t *_containers, int count )
{
  pickups.clear();
  invalid.clear();

  dataIsLatLon = true;
  std::string errorStr;
  bool dataHasError;

  for ( int i = 0; i < count; ++i ) {
    dataHasError = false;
    container_t c = _containers[i];

    if (i == 0) dataIsLatLon = isLatLon(c.x, c.y);

    if (dataIsLatLon and not isLatLon(c.x, c.y) ) {
      errorStr = "Container #" + numbertoString(c.id) +
                 ": expecting Lat/Lon coordinates, other was given";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };

    if (not dataIsLatLon and  isLatLon(c.x, c.y) ) {
      errorStr = "Container #" + numbertoString(c.id) +
                 ": expecting NOT lat/long coordinates, Lat/Lon was given";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };

    if (c.open > c.close) {
      errorStr = "Container #" + numbertoString(c.id) +
                 ": is Closing before it Opens";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };

    if (c.demand == 0) {
      errorStr = "Container #" + numbertoString(c.id) +
                 ": Is empty, Demand should be non zero positive";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };

    if (c.demand < 0) {
      errorStr = "Container #" + numbertoString(c.id) +
                 ": Demand <0, should have a non zero positive value";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };


    Trashnode node(c.id, c.x, c.y, c.open, c.close, c.service, c.demand,
                    c.sid);

    node.set_type(Twnode::kPickup);

    if (not dataHasError) {
      pickups.push_back(node);
    } else {
      invalid.push_back(node);
    }

  } // for
  return dataHasError;
}


bool Trash_check::CheckOtherlocs( otherloc_t *_otherlocs, int count )
{
  otherlocs.clear();
  bool dataHasError;
  std::string errorStr;

  for ( int i = 0; i < count; ++i ) {
    dataHasError = false;
    otherloc_t c = _otherlocs[i];

    if (dataIsLatLon and not isLatLon(c.x, c.y) ) {
      errorStr = "Other Location #" + numbertoString(c.id) +
                 ": expecting Lat/Lon coordinates, other was given";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };

    if (not dataIsLatLon and  isLatLon(c.x, c.y) ) {
      errorStr = "Other Location #" + numbertoString(c.id) +
                 ": expecting NOT lat/long coordinates, Lat/Lon was given";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };

    if (c.open > c.close) {
      errorStr = "Other Location #" + numbertoString(c.id) +
                 ": is Closing before it Opens";
      errorsFound.push_back(errorStr);
      dataHasError = true;
    };


    Trashnode node( c.id, c.x, c.y, c.open, c.close, 0, 0, -1 );

    if (  not dataHasError ) {
      otherlocs.push_back( node );
    } else {
      invalid.push_back( node );
    }
  }
  return dataHasError;
}




bool Trash_check::CheckVehicles( vehicle_t *_vehicles, int count )
{
  bool dataHasError = false;
  std::string errorStr;

  for ( int i = 0; i < count; ++i ) {
    dataHasError = false;
    vehicle_t v = _vehicles[i];

    if (not ( v.start_id >= 0 and v.dump_id >= 0 and v.end_id >= 0 and
              v.starttime >= 0 and v.starttime <= v.endtime and v.capacity > 0 and
              v.dumpservicetime >= 0 and v.vid >= 0 and otherlocs.hasId( v.start_id ) and
              otherlocs.hasId( v.dump_id ) and otherlocs.hasId( v.end_id ) ) ) {

      dataHasError = true;

      if (v.starttime < 0) {
        errorStr = "Truck #" + numbertoString(v.vid) + ": has negative start time";
        errorsFound.push_back(errorStr);
      };

      if (v.start_id < 0) {
        errorStr = "Truck #" + numbertoString(v.vid) +
                   ": has negative starting place id ";
        errorsFound.push_back(errorStr);
      } else {
        if (otherlocs.hasId(v.start_id) ) {
          errorStr = "Truck #" + numbertoString(v.vid) + ": Starting location not found ";
          errorsFound.push_back(errorStr);
        }
      };

      if (v.dump_id < 0) {
        errorStr = "Truck #" + numbertoString(v.vid) + ": has negative dump place id ";
        errorsFound.push_back(errorStr);
      } else {
        if (otherlocs.hasId(v.dump_id) ) {
          errorStr = "Truck #" + numbertoString(v.vid) + ": dump location not found ";
          errorsFound.push_back(errorStr);
        }
      };

      if (v.end_id < 0) {
        errorStr = "Truck #" + numbertoString(v.vid) +
                   ": has negative ending place id ";
        errorsFound.push_back(errorStr);
      } else {
        if (otherlocs.hasId(v.end_id) ) {
          errorStr = "Truck #" + numbertoString(v.vid) + ": ending location not found ";
          errorsFound.push_back(errorStr);
        }
      };


      if (v.starttime > v.endtime) {
        errorStr = "Truck #" + numbertoString(v.vid) +
                   ": shift ends before it starts (start time > end time)";
        errorsFound.push_back(errorStr);
      };

      if (v.capacity <= 0) {
        errorStr = "Truck #" + numbertoString(v.vid) +
                   ": has no capacity or has negative capacity)";
        errorsFound.push_back(errorStr);
      };


      if (v.dumpservicetime <= 0) {
        errorStr = "Truck #" + numbertoString(v.vid) +
                   ": has negative service time at the dump)";
        errorsFound.push_back(errorStr);
      };

    };

    Vehicle truck(v.vid, v.start_id, v.dump_id, v.end_id, v.capacity,
                  v.dumpservicetime, v.starttime, v.endtime, otherlocs);

    if (not dataHasError) {
      trucks.push_back(truck);
      depots.push_back(truck.getStartingSite());
      dumps.push_back(truck.getDumpSite());
      endings.push_back(truck.getEndingSite());
    } else {
      invalidTrucks.push_back( truck );
    }
  }

  return dataHasError;
}


bool Trash_check::isValid() const {
  return trucks.size()
         and depots.size()
         and dumps.size()
         and endings.size()
         and pickups.size()
         and otherlocs.size()
         and not invalid.size()
         and not invalidTrucks.size()
         and not errorsFound.size();
  ;
}

void Trash_check::whatIsWrong() {
  std::string errorStr;

  if ( not trucks.size() ) {
    errorStr = "No valid vehicles";
    errorsFound.push_back(errorStr);
  };

  if ( not depots.size() ) {
    errorStr = "No valid starting locations found";
    errorsFound.push_back(errorStr);
  };

  if ( not dumps.size() ) {
    errorStr = "No valid dumps found";
    errorsFound.push_back(errorStr);
  };

  if ( not endings.size() ) {
    errorStr = "No valid ending locations found";
    errorsFound.push_back(errorStr);
  };

  if ( not pickups.size() ) {
    errorStr = "No valid container locations found";
    errorsFound.push_back(errorStr);
  };

  if ( not otherlocs.size() ) {
    errorStr = "No valid other locations found";
    errorsFound.push_back(errorStr);
  };

};


std::string Trash_check::getErrorsString() const
{
  std::string errorStr = "";

  for (UINT i = 0; i < errorsFound.size(); i++)
    errorStr += errorsFound[i] + "\n";

  return errorStr;
}



