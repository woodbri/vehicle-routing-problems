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
#include <osrm/osrm.hpp>
#include <osrm/libosrm_config.hpp>


#include <climits>
#include <sstream>

#ifdef DOVRPLOG
#include "logger.h"
#endif


#include "osrmclient.h"

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif


OsrmClient *OsrmClient::p_osrm = NULL;
OSRM *OsrmClient::routing_machine = NULL;
bool OsrmClient::connectionAvailable = true;

OsrmClient::OsrmClient(const OsrmClient &other)
{
  connectionAvailable = other.connectionAvailable;
};

OsrmClient &OsrmClient::operator=(const OsrmClient &other)
{
  connectionAvailable = other.connectionAvailable;
};

/*!  * \brief The OsrmClient constructor.  */
OsrmClient::OsrmClient()
{
  if ( not connectionAvailable ) return;

#ifdef DOSTATS
  Timer timer;
#endif

  try {
    // No server paths needed (shared memory)
    ServerPaths server_paths;
    // Develop branch
    libosrm_config losrm_config;
    losrm_config.server_paths = server_paths;
    losrm_config.use_shared_memory = true;
    // Default
    losrm_config.max_locations_distance_table = 100;
    // Default
    losrm_config.max_locations_map_matching = -1;
    OsrmClient::routing_machine = new OSRM( losrm_config );
  } catch ( std::exception &e ) {
    status = -1;
    err_msg = std::string( "OsrmClient::OsrmClient caught exception: " ) + e.what();
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::OsrmClient Cumulative time", timer.duration() );
#endif
    connectionAvailable = false;
    return;
  };

  route_parameters.zoom_level = 18;
  route_parameters.print_instructions = true;
  route_parameters.alternate_route = false;
  route_parameters.geometry = false;
  route_parameters.compression = false;
  route_parameters.check_sum = UINT_MAX;
  route_parameters.service = "viaroute";
  route_parameters.output_format = "json";
  route_parameters.jsonp_parameter = "";
  route_parameters.language = "";

  status = 0;

  use = false;

  addPenalty = false;

#ifdef DOVRPLOG
  testOsrmClient();

#endif
#ifdef DOSTATS
  STATS->addto( "OsrmClient::OsrmClient Cumulative time", timer.duration() );

#endif
}



/*!
 * \brief Clear out any old points and reset the OsrmClient to a clean state.
 */
void OsrmClient::clear()
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

#ifdef DOSTATS
  Timer timer;
#endif
  route_parameters.coordinates.clear();
  route_parameters.hints.clear();
  route_parameters.geometry = false;
  route_parameters.compression = false;
  // C++11 only
  osrm_reply = {};
  err_msg = "";

  if ( status > 0 ) status = 0;

#ifdef DOSTATS
  STATS->addto( "OsrmClient::Clear Cumulative time", timer.duration() );
#endif
}


/*!
 * \brief Add a location in WGS84 to the OSRM request.
 * \param[in] lat The latitude of the location you want to add.
 * \param[in] lon The Longitude of the location you want to add.
 * \note route_parameters.coordinates is defined as std::vector<FixedPointCoordinate> in OSRM.h. COORDINATE_PRECISION is also defined in OSRM.h.
 */
void OsrmClient::addViaPoint( double lat, double lon )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

#ifdef DOSTATS
  Timer timer;
#endif
  FixedPointCoordinate p( lat * COORDINATE_PRECISION,
                          lon * COORDINATE_PRECISION );
  route_parameters.coordinates.push_back( p );
#ifdef DOSTATS
  STATS->addto( "OsrmClient::addViaPoint Cumulative time", timer.duration() );
#endif
}


/*!
 * \brief Add a \ref Node as a location to the OSRM request.
 * \param[in] node The Node to add, assumed to be in WGS84.
 */
void OsrmClient::addViaPoint( const Node &node )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

#ifdef DOSTATS
  Timer timer;
#endif
  addViaPoint( node.y(), node.x() );
  route_parameters.hints.push_back( node.hint() );
#ifdef DOSTATS
  STATS->addto( "OsrmClient::addViaPoint Cumulative time", timer.duration() );
#endif
}


/*!
 * \brief Add a path of \ref Node as locations to a the OSRM request.
 * \param[in] path A std::deque<Node> that you want to add.
 */
void OsrmClient::addViaPoints( const std::deque<Node> &path )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

  std::deque<Node>::const_iterator it;

  for ( it = path.begin(); it != path.end(); ++it )
    addViaPoint( *it );
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 , double lat2,
                              double lon2, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (2 points) " );
#endif
  clear();
  addViaPoint( lat1, lon1 );
  addViaPoint( lat2, lon2 );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 , double lat2,
                              double lon2, const  std::string &hint1, const std::string &hint2,
                              double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (2 points) " );
#endif
  clear();
  addViaPoint( lat1, lon1 );
  addViaPoint( lat2, lon2 );
  route_parameters.hints.push_back( hint1 );
  route_parameters.hints.push_back( hint2 );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (2 nodes) " );
#endif
  clear();
  addViaPoint( node1 );
  addViaPoint( node2 );
  route_parameters.hints.push_back( node1.hint() );
  route_parameters.hints.push_back( node2.hint() );

  if ( getOsrmViaroute() ) return  getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              const Node &node3, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (3 nodes) " );
#endif
  clear();
  addViaPoint( node1 );
  addViaPoint( node2 );
  addViaPoint( node3 );
  route_parameters.hints.push_back( node1.hint() );
  route_parameters.hints.push_back( node2.hint() );
  route_parameters.hints.push_back( node3.hint() );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              const Node &node3, const Node &node4, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (4 nodes) " );
#endif
  clear();
  addViaPoint( node1 );
  addViaPoint( node2 );
  addViaPoint( node3 );
  addViaPoint( node4 );
  route_parameters.hints.push_back( node1.hint() );
  route_parameters.hints.push_back( node2.hint() );
  route_parameters.hints.push_back( node3.hint() );
  route_parameters.hints.push_back( node4.hint() );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}



/*!
 * \brief Connect to the OSRM engine, issue the request and save the json response back in the object.
 * \return True if an error happened and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmViaroute()
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmViaRoute (does the work) " );
#endif

  if ( route_parameters.coordinates.size() < 2 ) {
    err_msg = "OsrmClient:getOsrmViaroute must be called with two ro more viapoints!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                  timer.duration() );
#endif
    return false;
  }

  if ( status == -1 ) {
    err_msg = "OsrmClient:getOsrmViaroute Failed to connect to server!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                  timer.duration() );
#endif
    return false;
  }

  err_msg = "";
  osrm_reply = {};

  try {
    routing_machine->RunQuery( route_parameters, osrm_reply );
  } catch ( std::exception &e ) {
    err_msg = std::string( "OsrmClient:getOsrmViaRoute caught exception: " )
              + e.what();
    connectionAvailable = false;
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                  timer.duration() );
#endif
    return false;
  }

  status = 1;
#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                timer.duration() );
#endif
  return true;
}


/*!
 * \brief Get the OSRM travel time for the requested route.
 * \param[out] time The OSRM travel time in decimal minutes.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmTime( double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmTime (does the work)" );
#endif

  if ( status != 1 or osrm_reply.values.size() == 0 ) {
    err_msg = "OsrmClient:getOsrmTime does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTime (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

  /*
  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmTime invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTime (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }
  */

  if ( not getTime( time ) ) {
#ifdef DOSTATS
    STATS->addto( "OsrmClient::getOsrmTime (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmTime (does the work) Cumulative time:",
                timer.duration() );
#endif
  return true;
}

bool OsrmClient::getOsrmPenalty( double &penalty )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmTurns" );
#endif

  if ( status != 1 or osrm_reply.values.size() == 0 ) {
    err_msg = "OsrmClient:getOsrmTurns does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumultaive time:",
                  timer.duration() );
#endif
    return false;
  }

  /*
  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmTurns (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumultaive time:",
                  timer.duration() );
#endif
    return false;
  }
  */

  if ( not getPenalty( penalty ) ) {
#ifdef DOSTATS
    STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumultaive time:",
                  timer.duration() );
#endif
    return false;
  }

#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumulative time:",
                timer.duration() );
#endif
  return true;
}


/*!
 * \brief Extract the geometry from the OSRM response if it was requested.
 * \param[out] geom A std::deque<Node> with each point in the path set as a \ref Node.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmGeometry( std::deque<Node> &geom )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmGeometry (does the work) " );
#endif

  if ( status != 1 or osrm_reply.values.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmGeometry does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  /*
  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmGeometry invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }
  */

  if ( not getGeom( geom ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::getOsrmGeometryText( std::string &geomText )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmGeometry (does the work) " );
#endif

  if ( status != 1 or osrm_reply.values.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmGeometry does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
}

  /*
  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmGeometry invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }
  */

  if ( not getGeomText( geomText ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::getOsrmHints( std::deque<std::string> &hints )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmHint (interface) " );
#endif

  if ( status != 1 or osrm_reply.values.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmHint (interface) does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  /*
  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmHint (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }
  */

  if ( not getHints( hints ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::testOsrmClient()
{
  if ( not use ) return false;

  if ( not connectionAvailable ) return false;

  std::deque<std::string> hints;

  if ( getStatus() == -1 ) {
#ifdef DOVRPLOG
    DLOG( WARNING ) << getErrorMsg();
#endif
    return false;
  }

  double penalty;
  bool oldPenalty = addPenalty;
  double time;

  // TODO Make this location a config value
  //34.890816,-56.165529
  if ( getOsrmTime( -34.8917, -56.167694, -34.890816, -56.165529, time ) ) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "test time:" << time;
#endif
  } else return false;

  getOsrmPenalty( penalty );

  if ( getOsrmHints( hints ) ) {
    std::string hint1 = hints[0];
    std::string hint2 = hints[1];

    if ( getOsrmTime( -34.8917, -56.167694, -34.890816, -56.165529,
                      hint1, hint2, time ) ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "test time:" << time;
#endif
    } else return false;
  } else
    return false;

  if ( not getOsrmPenalty( penalty ) )
    return false;

  addPenalty = true;

  if ( getOsrmTime( -34.8917, -56.167694, -34.890816, -56.165529, time ) ) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "test time: " << time;
#endif
  } else return false;

  getOsrmPenalty( penalty );

  if ( getOsrmHints( hints ) ) {
    std::string hint1 = hints[0];
    std::string hint2 = hints[1];

    if ( getOsrmTime( -34.8917, -56.167694, -34.890816, -56.165529, hint1, hint2,
                      time ) ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "test time: " << time;
#endif
    } else return false;
  } else return false;

  addPenalty = oldPenalty;

  return true;
};




// --------- private ----------------
/*!
 * \brief Parse the actual json document and extract the OSRM Travel time.
 * \param[out] time The OSRM travel time as decimal minutes.
 * \return False if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getTime( double &time )
{
  if ( not connectionAvailable ) return false;

  // find 'route_summary'
  auto got_rs = osrm_reply.values.find("route_summary");

  if ( got_rs == osrm_reply.values.end() ) {
    err_msg = "OsrmClient:getTime failed to find 'route_summary' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // find the 'total_time' in the 'route_summary'
  osrm::json::Object total_time = mapbox::util::get< osrm::json::Object >( osrm_reply.values["route_summary"] );
  auto got_tt = total_time.values.find("total_time");

  if ( got_tt == total_time.values.end() ) {
    err_msg = "OsrmClient:getTime failed to find 'total_time' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // extract the total_time and convert from seconds to minutes
  //time = ( double ) jsondoc["route_summary"]["total_time"].GetDouble() / 60.0;
  osrm::json::Number n = mapbox::util::get< osrm::json::Number >( total_time.values["total_time"] );
  time = n.value / 60.0;
  double penalty;

  if ( addPenalty and getPenalty( penalty ) ) time += penalty;

  return true;
}


/*!
 * \brief Parse the actual json document and extract the geometry Nodes
 * \param[out] geom A std::deque<Node> with each point in the path.
 * \return False if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getGeom( std::deque<Node> &geom )
{
  if ( not connectionAvailable ) return false;

  // clear the path
  geom.clear();

  // find the 'route_geometry' key in the response
  auto got_rg = osrm_reply.values.find("route_geometry");

  if ( got_rg == osrm_reply.values.end() ) {
    err_msg = "OsrmClient:getGeom failed to find 'route_geometry' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // "route_geometry" = [[lat,lon],[lat,lon], ... ]
  osrm::json::Array rg_arr = mapbox::util::get< osrm::json::Array > ( osrm_reply.values["route_geometry"] );

  for (auto it = rg_arr.values.begin() ; it != rg_arr.values.end(); ++it) {
    osrm::json::Array elem_arr = mapbox::util::get< osrm::json::Array > ( (*it) );
    osrm::json::Number xn = mapbox::util::get< osrm::json::Number > ( elem_arr.values[1] );
    osrm::json::Number yn = mapbox::util::get< osrm::json::Number > ( elem_arr.values[0] );
    Node n( xn.value, yn.value );
    geom.push_back( n );
  }

  return true;
}


/*!
 * \brief Parse the actual json document and extract the geometry text
 * \param[out] geomText A std::string which it the compressed geometry
 * \return False if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getGeomText( std::string &geomText )
{
  if ( not connectionAvailable ) return false;

  // find the 'route_geometry' key in the response
  auto got_rg = osrm_reply.values.find("route_geometry");

  if ( got_rg == osrm_reply.values.end() ) {
    err_msg = "OsrmClient:getGeomText failed to find 'geometry' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // "route_geometry" = [[lat,lon],[lat,lon], ... ]
  osrm::json::String osrm_str = mapbox::util::get< osrm::json::String > ( osrm_reply.values["route_geometry"] );
  geomText = osrm_str.value;
  return true;
}

/*!
 * \brief Parse the actual json document and extract the geometry text
 * \param[out] geomText A std::string which it the compressed geometry
 * \return False if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getHints( std::deque<std::string> &hints )
{
  if ( not connectionAvailable ) return false;

  hints.clear();

  // find 'hint_data'
  auto got_hd = osrm_reply.values.find("hint_data");

  if ( got_hd == osrm_reply.values.end() ) {
    err_msg = "OsrmClient:getHints (private) failed to find 'hint_data' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // find the 'locations' in the 'hint_data'
  osrm::json::Object hint_data = mapbox::util::get< osrm::json::Object >( osrm_reply.values["hint_data"] );
  auto got_loc = hint_data.values.find("locations");

  if ( got_loc == hint_data.values.end() ) {
    err_msg = "OsrmClient:getHints (private) failed to find 'locations' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  osrm::json::Array loc_arr = mapbox::util::get< osrm::json::Array >( hint_data.values["locations"] );

  for (auto it = loc_arr.values.begin() ; it != loc_arr.values.end(); ++it) {
    osrm::json::String str_elem = mapbox::util::get< osrm::json::String > ( (*it) );
    hints.push_back( str_elem.value );
  }

  return true;
}

/*!
 * \brief Parse the actual json document and extract the geometry text
 * \param[out] geomText A std::string which it the compressed geometry
 * \return False if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getPenalty( double &penalty )   //in munutes
{
  if ( not connectionAvailable ) return false;

  int turn;

  std::string  trace;

  penalty = 0;

  // find the 'route_geometry' key in the response
  auto got_ri = osrm_reply.values.find("route_instructions");

  if ( got_ri == osrm_reply.values.end() ) {
    err_msg = "OsrmClient:getTurns (private) failed to find 'route_instructions' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // "route_instructions" = [["10","Avenida General Rivera",42,0,13,"42m","NE",57,1], ...]
  osrm::json::Array ri_arr = mapbox::util::get< osrm::json::Array > ( osrm_reply.values["route_instructions"] );

  for (auto it = ri_arr.values.begin() ; it != ri_arr.values.end(); ++it) {
    osrm::json::Array elem_arr = mapbox::util::get< osrm::json::Array > ( (*it) );
    osrm::json::String ti = mapbox::util::get< osrm::json::String > ( elem_arr.values[0] );
    turn = std::stoi( ti.value );
    trace = ti.value;
#ifdef DOVRPLOG
    DLOG( INFO ) << "InstructionsData " << trace;
    DLOG( INFO ) << "Instruction " << turn;
#endif

    switch ( turn ) {
    case 2: penalty += 0.05; break; //slight right

    case 3: penalty += 0.10; break; //right

    case 4: penalty += 0.3 ; break; //sharp right

    case 5: penalty += 1;  break; //uturn

    case 8: penalty += 0.05; break; //slight left

    case 7: penalty += 0.10; break; //left

    case 6: penalty += 0.3 ; break; //sharp left
    }

  }

#ifdef DOVRPLOG
  DLOG( INFO ) << "Penalty " << penalty;
#endif
  return true;
}

