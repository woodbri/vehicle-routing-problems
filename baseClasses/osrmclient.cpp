
#include "Library/OSRM.h"
#include "osrmclient.h"

#ifdef DOSTATS
#include "timer.h"
#endif


/*!
 * \brief The OsrmClient constructor.
 */
OsrmClient::OsrmClient() {
    try {
        // create shared memory connection to the server
        //ServerPaths server_paths;
        //routing_machine = new OSRM( server_paths, true );

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
    }
    catch ( std::exception & e ) {
        status = -1;
        //routing_machine = NULL;
        err_msg = std::string("OsrmClient: caught exception: ") + e.what();
    };
}


/*!
 * \brief Clear out any old points and reset the OsrmClient to a clean state.
 */
void OsrmClient::clear() {
    route_parameters.coordinates.clear();
    route_parameters.geometry = false;
    httpContent = "";
    err_msg = "";
    if (status > 0) status = 0;
}


/*!
 * \brief Add a location in WGS84 to the OSRM request.
 * \param[in] lat The latitude of the location you want to add.
 * \param[in] lon The Longitude of the location you want to add.
 * \note route_parameters.coordinates is defined as std::vector<FixedPointCoordinate> in OSRM.h. COORDINATE_PRECISION is also defined in OSRM.h.
 */
void OsrmClient::addViaPoint( double lat, double lon ) {
    FixedPointCoordinate p( lat * COORDINATE_PRECISION,
                            lon * COORDINATE_PRECISION );
    route_parameters.coordinates.push_back( p );
}


/*!
 * \brief Add a \ref Node as a location to the OSRM request.
 * \param[in] node The Node to add, assumed to be in WGS84.
 */
void OsrmClient::addViaPoint( const Node &node ) {
    addViaPoint( node.gety(), node.getx() );
}


/*!
 * \brief Add a path of \ref Node as locations to a the OSRM request.
 * \param[in] path A std::deque<Node> that you want to add.
 */
void OsrmClient::addViaPoints( const std::deque<Node> &path ) {
    std::deque<Node>::const_iterator it;
    for ( it = path.begin(); it != path.end(); ++it )
        addViaPoint( *it );
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 ,double lat2, double lon2, double &time ) {
    clear();
    addViaPoint(lat1,lon1);
    addViaPoint(lat2,lon2);
    if (getOsrmViaroute()) return getOsrmTime(time);
    return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2, double &time ) {
    clear();
    addViaPoint(node1);
    addViaPoint(node2);
    if (getOsrmViaroute()) return getOsrmTime(time);
    return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2, const Node &node3, double &time ) {
    clear();
    addViaPoint(node1);
    addViaPoint(node2);
    addViaPoint(node3);
    if (getOsrmViaroute()) return getOsrmTime(time);
    return false;
}


/*!
 * \brief Connect to the OSRM engine, issue the request and save the json response back in the object.
 * \return True if an error happened and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmViaroute() {
    if ( route_parameters.coordinates.size() < 2 ) {
        err_msg = "OsrmClient: getOsrmViaroute must be called with two ro more viapoints!";
        return false;
    }
    if ( status == -1 ) {
        err_msg = "OsrmClient: Failed to connect to server!";
        return false;
    }

    try {
        // create shared memory connection to the server
        ServerPaths server_paths;
        OSRM routing_machine( server_paths, true );

        err_msg = "";
        http::Reply osrm_reply;

        routing_machine.RunQuery( route_parameters, osrm_reply );

        httpContent = "";
        std::vector<std::string>::iterator sit;
        for ( sit=osrm_reply.content.begin();
              sit != osrm_reply.content.end();
              ++sit )
            httpContent += *sit;

        status = 1;

        return true;
    }
    catch ( std::exception & e ) {
        err_msg = std::string("OsrmClient: caught exception: ")
                + e.what();
        return false;
    }
}


/*!
 * \brief Get the OSRM travel time for the requested route.
 * \param[out] time The OSRM travel time in decimal minutes.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmTime( double &time ) {
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient: does not have a valid OSRM response!";
        return false;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient: invalid json document in OSRM response!";
        return false;
    }

    if ( getTime( jtree, time ) ) {
        json_object_put( jtree );
        return false;
    }

    json_object_put( jtree );
    return true;
}


/*!
 * \brief Extract the geometry from the OSRM response if it was requested.
 * \param[out] geom A std::deque<Node> with each point in the path set as a \ref Node.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmGeometry( std::deque<Node> &geom ) {
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient: does not have a valid OSRM response!";
        return false;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient: invalid json document in OSRM response!";
        return false;
    }

    if ( getGeom( jtree, geom ) ) {
        json_object_put( jtree );
        return false;
    }

    json_object_put( jtree );
    return true;
}


int OsrmClient::testOsrmClient() {
    Timer t0;
    double time;
    dump();
    if (getStatus() == -1) {
        std::cout << getErrorMsg() << std::endl;
        return -1;
    }
    if (getOsrmTime(-34.88124, -56.19048,-34.89743, -56.12447,time) )
         std::cout << "SUCCESSSSS  getOsrmTime: " << time << std::endl;
    dump();
    std::cout << "duration: " << t0.duration() << std::endl;
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
    struct json_object * jobj;

    // find the 'route_sammary' key in the response
    jobj = json_object_object_get( jtree, "route_summary" );
    if ( not jobj ) {
        err_msg = "OsrmClient: failed to find 'route_summary' key in OSRM response!";
        return false;
    }

    // find the 'total_time' in the 'route_summary'
    jobj = json_object_object_get( jobj, "total_time" );
    if ( not jobj ) {
        err_msg = "OsrmClient: failed to find 'total_time' key in OSRM response!";
        return false;
    }

    // extract the total_time and convert to seconds
    time = (double) json_object_get_int( jobj ) / 60.0;
    return true;
}


/*!
 * \brief Parse the actual json document and extract the geometry Nodes
 * \param[in] jtree The json parse tree pointer.
 * \param[out] geom A std::deque<Node> with each point in the path.
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getGeom( struct json_object *jtree, std::deque<Node> &geom ) {
    struct json_object * jobj;

    // clear the path
    geom.clear();

    // find the route 'geometry' key in the response
    jobj = json_object_object_get( jtree, "route_geometry" );
    if ( not jobj ) {
        err_msg = "OsrmClient: failed to find 'geometry' key in OSRM response!";
        return false;
    }

    int numPnts = json_object_array_length( jobj );

    //std::cout << "route_geometry: type: " << json_object_get_type(jobj)
    //          << ", size: " << numPnts << "\n";

    json_object * jval;
    for (int i=0; i<numPnts; ++i) {
        jval = json_object_array_get_idx(jobj, i);
        //std::cout << "jval: type: " << json_object_get_type(jval)
        //          << ", size: " << json_object_array_length(jval) << "\n";

        json_object * j0 = json_object_array_get_idx(jval, 0);
        //std::cout << "j0: type: " << json_object_get_type(j0) << "\n";

        json_object * j1 = json_object_array_get_idx(jval, 1);
        //std::cout << "j1: type: " << json_object_get_type(j1) << "\n";

        double x = json_object_get_double( j0 );
        double y = json_object_get_double( j1 );
        Node n(x, y);
        geom.push_back( n );
    }

    return true;
}


