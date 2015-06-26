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

#include "trashprob.h"

#ifdef DOVRPLOG
#include "logger.h"
#endif


TrashProb::TrashProb(  container_t *p_containers, unsigned int container_count,
                       otherloc_t *p_otherlocs, unsigned int otherloc_count,
                       ttime_t *p_ttimes, unsigned int ttime_count,
                       vehicle_t *p_vehicles, unsigned int vehicle_count,
                       unsigned int check)
{

  Bucket nodes;
  Bucket intersection;
#ifdef VRPMINTRACE
  DLOG( INFO ) << "trashProb LoadProblem -------------from sql data---------";
#endif


  // read the nodes
  //int cnt = 0;
  //int nid = 0;
  twc->emptiedTruck = false;

  addContainers(p_containers, container_count);

  if ( not pickups.size() ) {
#ifdef VRPMINTRACE
    DLOG(INFO) << getErrorsString();
#endif
    return;
  }

  addOtherlocs( p_otherlocs, otherloc_count );

  if ( not otherlocs.size() ) {
#ifdef VRPMINTRACE
    DLOG(INFO) << getErrorsString();
#endif
    return;
  }

  intersection = otherlocs * pickups;
  invalid = invalid + intersection;
  pickups = pickups - intersection;
  nodes = nodes - intersection;

  std::string errorStr;

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


  nodes = pickups + otherlocs;
  nodes.push_back( C );

  int id = 0;

  for ( UINT i = 0; i < nodes.size(); i++ ) {
    nodes[i].set_nid( i );
    id = nodes[i].id();

    if ( pickups.hasId( id ) )
      pickups[ pickups.posFromId( id ) ].set_nid( i );
    else if ( otherlocs.hasId( id ) )
      otherlocs[ otherlocs.posFromId( id ) ].set_nid( i );
  };

  C = nodes.back();

  datanodes = nodes;

  twc->loadAndProcess_distance( p_ttimes, ttime_count, datanodes, invalid );

  addVehicles( p_vehicles, vehicle_count );

  if (not trucks.size()) {
#ifdef VRPMINTRACE
    DLOG(INFO) << getErrorsString();
#endif
    return;
  };

#ifdef OSRMCLIENT
  twc->setHints( dumps );
  twc->setHints( nodes );
  twc->setHints( depots );
  twc->setHints( pickups );
  twc->setHints( endings );

  if (not check) twc->fill_travel_time_onTrip();
  twc->settCC( C, pickups );
#endif  // OSRMCLIENT


  assert( trucks.size() and depots.size() and dumps.size() and endings.size() );

  for ( UINT i = 0; i < trucks.size(); i++ ) {
    trucks[i].setInitialValues( C, pickups );
  }


#ifdef VRPMAXTRACE
  DLOG ( INFO ) << "Average Node";
  C.dump();
  nodes.dump( "nodes" );
  dumps.dump( "dumps" );
  depots.dump( "depots" );
  pickups.dump( "pickups" );
  endings.dump( "endings" );
  datanodes.dump( "datanodes" );
  invalid.dump( "invalid" );
  DLOG( INFO ) << "TRUCKS";

  for ( UINT i = 0; i < trucks.size(); i++ ) trucks[i].tau();

  DLOG( INFO ) << "INVALID TRUCKS";

  for ( UINT i = 0; i < invalidTrucks.size(); i++ ) invalidTrucks[i].tau();

#endif
#ifdef VRPMAXTRACE
  twc->dump();
#endif
}

bool isLatLon(double x, double y)
{
  return       -180.0 <= x and x <= 180.0
               and  - 180.0 <= y and y <= 180.0;
}

void TrashProb::addContainers( container_t *_containers, int count )
{
  pickups.clear();
  invalid.clear();

  double st, op, cl, dm, x, y;
  st = op = cl = dm = x = y = 0;
  dataIsLatLon = false;
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


    Trashnode node( c.id, c.x, c.y, c.open, c.close, c.service, c.demand,
                    c.sid ); //get it out of the cycle

    node.set_type( Twnode::kPickup );

    if ( not dataHasError ) {
      pickups.push_back( node );
      st += node.serviceTime();
      op += node.opens();
      cl += node.closes();
      dm += node.demand();
      x += node.x();
      y += node.y();
    } else invalid.push_back( node );

  }

  st = st / pickups.size();
  op = op / pickups.size();
  cl = cl / pickups.size();
  dm = dm / pickups.size();
  x = x / pickups.size();
  y = y / pickups.size();
  C.set( -1, -1, x, y, dm, op, cl, st );
}


void TrashProb::addOtherlocs( otherloc_t *_otherlocs, int count )
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
}



void TrashProb::addVehicles( vehicle_t *_vehicles, int count )
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


    Vehicle truck( v.vid, v.start_id, v.dump_id, v.end_id, v.capacity,
                   v.dumpservicetime, v.starttime, v.endtime, otherlocs );

    if ( not dataHasError ) {
      trucks.push_back( truck );
      depots.push_back( truck.getStartingSite() );
      dumps.push_back( truck.getDumpSite() );
      endings.push_back( truck.getEndingSite() );
    } else {
      invalidTrucks.push_back( truck );
    }
  }

}


bool TrashProb::isValid() const
{
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

void  TrashProb::whatIsWrong()
{
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


std::string TrashProb::getErrorsString() const
{
  std::string errorStr = "";

  for (UINT i = 0; i < errorsFound.size(); i++)
    errorStr += errorsFound[i] + "\n";

  return errorStr;
}

#if 0
std::string TrashProb::whatIsWrong() const
{
  std::ostringstream wiw( std::ostringstream::ate );

  if ( not trucks.size() ) wiw << "No valid vehicles\n";

  if ( not depots.size() ) wiw << "No valid starting locations found\n";

  if ( not dumps.size() ) wiw << "No valid dumps found\n";

  if ( not endings.size() ) wiw << "No valid ending locations found\n";

  if ( not pickups.size() ) wiw << "No valid container locations found\n";

  if ( not otherlocs.size() ) wiw << "No valid other locations found\n";

  if ( invalid.size() ) {
    wiw << "The following nodes are invalid: ";

    for ( int i = 0; i < invalid.size(); ++i )
      wiw << invalid[i].getid() << " ";

    wiw << "\n";
  }

  if ( invalidTrucks.size() ) {
    wiw << "The following vehicles are invalid: ";

    for ( int i = 0; i < invalidTrucks.size(); ++i )
      wiw << invalidTrucks[i].getVid() << " ";

    wiw << "\n";
  }

  return wiw.str();
}
#endif


