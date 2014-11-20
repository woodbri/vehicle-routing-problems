#include "Library/OSRM.h"

#include "osrmclient.h"

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif


OsrmClient *OsrmClient::p_osrm=NULL;
OSRM * OsrmClient::routing_machine= NULL;
bool OsrmClient::connectionAvailable=true;

/*!
 * \brief The OsrmClient constructor.
 */
OsrmClient::OsrmClient(){
    if (not connectionAvailable) return;
    #ifdef DOSTATS 
    Timer timer;
    #endif
    try {
        ServerPaths server_paths;
	OsrmClient::routing_machine = new OSRM(server_paths, true);
    }
    catch ( std::exception & e ) {
        status = -1;
        err_msg = std::string("OsrmClient::OsrmClient caught exception: ") + e.what();
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
        STATS->addto("OsrmClient::OsrmClient Cumulative time", timer.duration());
	#endif
	connectionAvailable=false;
	return;
    };

        route_parameters.zoomLevel = 18;
        route_parameters.printInstructions = false;
        route_parameters.alternateRoute = false;
        route_parameters.geometry = false;
        route_parameters.compression = false;
        route_parameters.checkSum = UINT_MAX;
        route_parameters.service = "viaroute";
        route_parameters.outputFormat = "json";
        route_parameters.jsonpParameter = "";
        route_parameters.language = "";
        status = 0;
	#ifndef LOG
	testOsrmClient();
	#endif
	#ifdef DOSTATS 
        STATS->addto("OsrmClient::OsrmClient Cumulative time", timer.duration());
	#endif
    }



/*!
 * \brief Clear out any old points and reset the OsrmClient to a clean state.
 */
void OsrmClient::clear() {
    if (not connectionAvailable) return;
    #ifdef DOSTATS 
    Timer timer;
    #endif
    route_parameters.coordinates.clear();
    route_parameters.hints.clear();
    route_parameters.geometry = false;
    httpContent = "";
    err_msg = "";
    if (status > 0) status = 0;
    #ifdef DOSTATS 
    STATS->addto("OsrmClient::Clear Cumulative time", timer.duration());
    #endif
}


/*!
 * \brief Add a location in WGS84 to the OSRM request.
 * \param[in] lat The latitude of the location you want to add.
 * \param[in] lon The Longitude of the location you want to add.
 * \note route_parameters.coordinates is defined as std::vector<FixedPointCoordinate> in OSRM.h. COORDINATE_PRECISION is also defined in OSRM.h.
 */
void OsrmClient::addViaPoint( double lat, double lon ) {
    if (not connectionAvailable) return;
    #ifdef DOSTATS 
    Timer timer;
    #endif
    FixedPointCoordinate p( lat * COORDINATE_PRECISION,
                            lon * COORDINATE_PRECISION );
    route_parameters.coordinates.push_back( p );
    #ifdef DOSTATS 
    STATS->addto("OsrmClient::addViaPoint Cumulative time", timer.duration());
    #endif
}


/*!
 * \brief Add a \ref Node as a location to the OSRM request.
 * \param[in] node The Node to add, assumed to be in WGS84.
 */
void OsrmClient::addViaPoint( const Node &node ) {
    if (not connectionAvailable) return;
    #ifdef DOSTATS 
    Timer timer;
    #endif
    addViaPoint( node.gety(), node.getx() );
    route_parameters.hints.push_back( node.getHint() );
    #ifdef DOSTATS 
    STATS->addto("OsrmClient::addViaPoint Cumulative time", timer.duration());
    #endif
}


/*!
 * \brief Add a path of \ref Node as locations to a the OSRM request.
 * \param[in] path A std::deque<Node> that you want to add.
 */
void OsrmClient::addViaPoints( const std::deque<Node> &path ) {
    if (not connectionAvailable) return;
    std::deque<Node>::const_iterator it;
    for ( it = path.begin(); it != path.end(); ++it )
        addViaPoint( *it );
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 ,double lat2, double lon2, double &time ) {
    #ifdef DOSTATS 
    STATS->inc("OsrmClient::getOsrmTime (2 points) ");
    #endif
    if (not connectionAvailable) return false;
    clear();
    addViaPoint(lat1,lon1);
    addViaPoint(lat2,lon2);
    if (getOsrmViaroute()) return getOsrmTime(time);
    return false;
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 ,double lat2, double lon2,const  std::string &hint1,const std::string &hint2, double &time ) {
    #ifdef DOSTATS 
    STATS->inc("OsrmClient::getOsrmTime (2 points) ");
    #endif
    if (not connectionAvailable) return false;
    clear();
    addViaPoint(lat1,lon1);
    addViaPoint(lat2,lon2);
    route_parameters.hints.push_back( hint1 );
    route_parameters.hints.push_back( hint2 );
    if (getOsrmViaroute()) return getOsrmTime(time);
    return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2, double &time ) {
    if (not connectionAvailable) return false;
    #ifdef DOSTATS 
    STATS->inc("OsrmClient::getOsrmTime (2 nodes) ");
    #endif
    clear();
    addViaPoint(node1);
    addViaPoint(node2);
    route_parameters.hints.push_back( node1.getHint() );
    route_parameters.hints.push_back( node2.getHint() );
    if (getOsrmViaroute()) return  getOsrmTime(time);
    return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2, const Node &node3, double &time ) {
    if (not connectionAvailable) return false;
    #ifdef DOSTATS 
    STATS->inc("OsrmClient::getOsrmTime (3 nodes) ");
    #endif
    clear();
    addViaPoint(node1);
    addViaPoint(node2);
    addViaPoint(node3);
    route_parameters.hints.push_back( node1.getHint() );
    route_parameters.hints.push_back( node2.getHint() );
    route_parameters.hints.push_back( node3.getHint() );
    if (getOsrmViaroute()) return getOsrmTime(time);
    return false;
}


/*!
 * \brief Connect to the OSRM engine, issue the request and save the json response back in the object.
 * \return True if an error happened and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmViaroute() {
    if (not connectionAvailable) return false;
    #ifdef DOSTATS 
    Timer timer;
    STATS->inc("OsrmClient::getOsrmViaRoute (does the work) ");
    #endif

    if ( route_parameters.coordinates.size() < 2 ) {
        err_msg = "OsrmClient:getOsrmViaroute must be called with two ro more viapoints!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
        STATS->addto("OsrmClient::getOsrmViaRoute (does the work) Cumulative time", timer.duration());
	#endif
        return false;
    }
    if ( status == -1 ) {
        err_msg = "OsrmClient:getOsrmViaroute Failed to connect to server!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
        STATS->addto("OsrmClient::getOsrmViaRoute (does the work) Cumulative time", timer.duration());
	#endif
        return false;
    }
    err_msg = "";
    http::Reply osrm_reply;

    try {
        routing_machine->RunQuery( route_parameters, osrm_reply );
    }
    catch ( std::exception & e ) {
        err_msg = std::string("OsrmClient:getOsrmViaRoute caught exception: ")
                + e.what();
	connectionAvailable=false;
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
        STATS->addto("OsrmClient::getOsrmViaRoute (does the work) Cumulative time", timer.duration());
	#endif
        return false;
    }


        //routing_machine.RunQuery( route_parameters, osrm_reply );

        httpContent = "";
        std::vector<std::string>::iterator sit;
        for ( sit=osrm_reply.content.begin();
              sit != osrm_reply.content.end();
              ++sit )
            httpContent += *sit;

        status = 1;
	#ifdef DOSTATS 
        STATS->addto("OsrmClient::getOsrmViaRoute (does the work) Cumulative time", timer.duration());
	#endif
        return true;
}


/*!
 * \brief Get the OSRM travel time for the requested route.
 * \param[out] time The OSRM travel time in decimal minutes.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmTime( double &time ) {
    if (not connectionAvailable) return false;
    #ifdef DOSTATS 
    Timer timer;
    STATS->inc("OsrmClient::getOsrmTime (does the work) ");
    #endif
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient:getOsrmTime does not have a valid OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
        STATS->addto("OsrmClient::getOsrmTime (does the work) Cumultaive time:", timer.duration());
	#endif
        return false;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient:getOsrmTime invalid json document in OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
        STATS->addto("OsrmClient::getOsrmTime (does the work) Cumultaive time:", timer.duration());
	#endif
        return false;
    }

    if ( not getTime( jtree, time ) ) {
        json_object_put( jtree );
	#ifdef DOSTATS 
        STATS->addto("OsrmClient::getOsrmTime (does the work) Cumultaive time:", timer.duration());
	#endif
        return false;
    }

    json_object_put( jtree );
    #ifdef DOSTATS 
    STATS->addto("Cumulative OsrmClient::getOsrmTime (does the work) time:", timer.duration());
    #endif
    return true;
}


/*!
 * \brief Extract the geometry from the OSRM response if it was requested.
 * \param[out] geom A std::deque<Node> with each point in the path set as a \ref Node.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmGeometry( std::deque<Node> &geom ) {
    if (not connectionAvailable) return false;
    #ifdef DOSTATS 
    Timer timer;
    STATS->inc("OsrmClient::getOsrmGeometry (does the work) ");
    #endif
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient::getOsrmGeometry does not have a valid OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
	#endif
        return false;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient:getOsrmGeometry invalid json document in OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
	#endif
        return false;
    }

    if ( not getGeom( jtree, geom ) ) {
        json_object_put( jtree );
        return false;
    }

    json_object_put( jtree );
    return true;
}

bool OsrmClient::getOsrmHints( std::deque<std::string> &hints ) {
    if (not connectionAvailable) return false;
    #ifdef DOSTATS 
    Timer timer;
    STATS->inc("OsrmClient::getOsrmHint (interface) ");
    #endif
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient::getOsrmHint (interface) does not have a valid OSRM response!";
        #ifdef DOSTATS 
        STATS->inc(err_msg);
        #endif
        return false;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient:getOsrmHint (interface) invalid json document in OSRM response!";
        #ifdef DOSTATS 
        STATS->inc(err_msg);
        #endif
        return false;
    }

    if ( not getHints( jtree, hints ) ) {
        json_object_put( jtree );
        return false;
    }

    json_object_put( jtree );
    return true;
}


int OsrmClient::testOsrmClient() {
    if (not connectionAvailable) return 0;
    Timer t0;
    double time;
    std::deque<std::string> hints;
    dump();
    if (getStatus() == -1) {
        std::cout << getErrorMsg() << std::endl;
        return -1;
    }
    for(int j=0;j<100;j++) {
    	t0.restart();
    	if (getOsrmTime(-34.88124, -56.19048,-34.89743, -56.12447,time) )
          std::cout << "SUCCESSSSS  getOsrmTime: " << time << std::endl;
    	STATS->addto("OsrmClient::testOsrmClient (no hints) time:", t0.duration());
    	if (getOsrmHints(hints)) {
	//for (int i=0;i<hints.size();i++) std::cout<<"hint "<<i<<": "<<hints[i]<<"\n";
	  std::string hint1= hints[0];
	  std::string hint2= hints[1];
          t0.restart();
          if (getOsrmTime(-34.88124, -56.19048,-34.89743, -56.12447,hint1,hint2,time) )
               std::cout << "YET ANOTHER SUCCESSSSS  getOsrmTime: " << time << std::endl;
    	STATS->addto("OsrmClient::testOsrmClient ( with hints) time:", t0.duration());
    }
    }
    dump();
    return 0;
};




// --------- private ----------------


/*!
 * \brief Parse the actual json document and extract the OSRM Travel time.
 * \param[in] jtree The json parse tree pointer.
 * \param[out] time The OSRM travel time as decimal minutes.
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getTime( struct json_object *jtree, double &time ) {
    if (not connectionAvailable) return false;
    struct json_object * jRouteSummary;
    struct json_object * jTime;

    // find the 'route_sammary' key in the response
    jRouteSummary = json_object_object_get( jtree, "route_summary" );
    if ( not jRouteSummary ) {
        err_msg = "OsrmClient:getTime failed to find 'route_summary' key in OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
	#endif
        return false;
    }

    // find the 'total_time' in the 'route_summary'
    jTime = json_object_object_get( jRouteSummary, "total_time" );
    if ( not jTime ) {
        err_msg = "OsrmClient:getTime failed to find 'total_time' key in OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
	#endif
        json_object_put( jRouteSummary );
        return false;
    }

    // extract the total_time and convert to seconds
    time = (double) json_object_get_int( jTime ) / 60.0;
    json_object_put( jRouteSummary );
    json_object_put( jTime );
    return true;
}


/*!
 * \brief Parse the actual json document and extract the geometry Nodes
 * \param[in] jtree The json parse tree pointer.
 * \param[out] geom A std::deque<Node> with each point in the path.
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getGeom( struct json_object *jtree, std::deque<Node> &geom ) {
    if (not connectionAvailable) return false;
    struct json_object * jobj;

    // clear the path
    geom.clear();

    // find the route 'geometry' key in the response
    jobj = json_object_object_get( jtree, "route_geometry" );
    if ( not jobj ) {
        err_msg = "OsrmClient:getGeom failed to find 'geometry' key in OSRM response!";
	#ifdef DOSTATS 
 	STATS->inc(err_msg);
	#endif
        json_object_put( jobj );
        return false;
    }

    int numPnts = json_object_array_length( jobj );

    //std::cout << "route_geometry: type: " << json_object_get_type(jobj)
    //          << ", size: " << numPnts << "\n";

    json_object *jval;
    json_object *j0;
    json_object *j1;
    double x;
    double y;
    for (int i=0; i<numPnts; ++i) {
        jval = json_object_array_get_idx(jobj, i);
        //std::cout << "jval: type: " << json_object_get_type(jval)
        //          << ", size: " << json_object_array_length(jval) << "\n";

        j0 = json_object_array_get_idx(jval, 0);
        //std::cout << "j0: type: " << json_object_get_type(j0) << "\n";

        j1 = json_object_array_get_idx(jval, 1);
        //std::cout << "j1: type: " << json_object_get_type(j1) << "\n";

        x = json_object_get_double( j0 );
        y = json_object_get_double( j1 );
        Node n(x, y);
        geom.push_back( n );
        json_object_put( j0 );
        json_object_put( j1 );
        json_object_put( jval );
    }

    json_object_put( jobj );
    return true;
}

bool OsrmClient::getHints( struct json_object *jtree, std::deque<std::string> &hints ) {
    if (not connectionAvailable) return false;
    struct json_object *jHintData;
    struct json_object *jLocations;
    hints.clear();

    // find the route 'hint_data' key in the response
    jHintData = json_object_object_get( jtree, "hint_data" );
    if ( not jHintData ) {
        err_msg = "OsrmClient:getHints (private)  failed to find 'hint_data' key in OSRM response!";
        #ifdef DOSTATS 
        STATS->inc(err_msg);
        #endif
        return false;
    }

    jLocations = json_object_object_get( jHintData, "locations" );
    if ( not jHintData ) {
        err_msg = "OsrmClient:getHints (private)  failed to find 'locations' key in OSRM response!";
        #ifdef DOSTATS 
        STATS->inc(err_msg);
        #endif
        json_object_put( jHintData );
        return false;
    }


    json_object *jHint;
    std::string hint;

    for (int i=0; i<route_parameters.coordinates.size(); i++) {
        jHint = json_object_array_get_idx(jLocations, i);

        hint = json_object_get_string (jHint);
        hints.push_back( hint );

        json_object_put( jHint );
    }

    json_object_put( jLocations );
    json_object_put( jHintData );
    return true;
}

