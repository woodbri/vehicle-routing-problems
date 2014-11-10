
#include "osrmclient.h"


OsrmClient::OsrmClient() {
    try {
        // create shared memory connection to the server
        ServerPaths server_paths;
        routing_machine = OSRM( server_paths, true );

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
        err_msg = std::string("OsrmClient: cauth exception: ") + e.what();
    };
}


void OsrmClient::clear() {
    route_parameters.coordinates.clear();
    httpContent = "";
    err_msg = "";
    if (status > 0) status = 0;
}


void OsrmClient::addViaPoint( double lat, double lon ) {
    FixedPointCoordinate p( lat * COORDINATE_PRECISION,
                            lon * COORDINATE_PRECISION );
    route_parameters.coordinates.push_back( p );
}


void OsrmClient::addViaPoint( const Node &node ) {
    addViaPoint( node.gety(), node.getx() );
}


void OsrmClient::addViaPoints( const std::deque<Node> &path ) {
    std::deque<Node>::const_iterator it;
    for ( it = path.begin(); it != path.end(); ++it )
        addViaPoint( *it );
}


bool OsrmClient::getOsrmViaroute( bool wantGeom ) {
    if ( viaPoints.size() < 2 ) {
        err_msg = "OsrmClient: getOsrmViaroute must be called with two ro more viapoints!";
        return false;
    }

    try {
        err_msg = "";
        http::Reply osrm_reply;

        routing_machine->RunQuery( route_parameters, osrm_reply );

        httpContent = "";
        std::vector<std::string>::iterator sit;
        for ( sit=osrm_reply.content.begin();
              sit != osrm_reply.content.end();
              ++sit )
            httpContent += *sit;

        status = 1;

        return false;
    }
    catch ( std::exception & e ) {
        err_msg = std::string("OsrmClient: caught exception: ")
                + e.what();
        return true;
    }
}


bool OsrmClient::getOsrmTime( double &time ) {
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient: does not have a valid OSRM response!";
        return true;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient: invalid json document in OSRM response!";
        return true;
    }

    if ( getTime( jtree, time ) ) {
        json_object_put( jtree );
        return true;
    }

    json_object_put( jtree );
    return false;
}


bool OsrmClient::getOsrmGeometry( std::deque<Node> &geom ) {
    if ( status != 1 or httpContent.size() == 0 ) {
        err_msg = "OsrmClient: does not have a valid OSRM response!";
        return true;
    }

    struct json_object * jtree = NULL;
    jtree = json_tokener_parse( httpContent.c_str() );
    if ( not jtree ) {
        err_msg = "OsrmClient: invalid json document in OSRM response!";
        return true;
    }

    if ( getGeom( jtree, geom ) ) {
        json_object_put( jtree );
        return true;
    }

    json_object_put( jtree );
    return false;
}

// --------- private ----------------

bool OsrmClient::getTime( struct json_object *jtree, double &time ) {
    struct json_object * jobj;

    // find the 'route_sammary' key in the response
    jobj = json_object_object_get( jtree, "route_summary" );
    if ( not jobj ) {
        err_msg = "OsrmClient: failed to find 'route_summary' key in OSRM response!";
        return true;
    }

    // find the 'total_time' in the 'route_summary'
    jobj = json_object_object_get( jobj, "total_time" );
    if ( not jobj ) {
        err_msg = "OsrmClient: failed to find 'total_time' key in OSRM response!";
        return true;
    }

    // extract the total_time and convert to seconds
    time = (double) json_object_get_int( jobj ) / 60.0;
    return false;
}


bool OsrmClient::getGeom( struct json_object *jtree, std::deque<Node> &geom ) {
    struct json_object * jobj;

    // clear the path
    geom.clear();

    // find the route 'geometry' key in the response
    jobj = json_object_object_get( jtree, "geometry" );
    if ( not jobj ) {
        err_msg = "OsrmClient: failed to find 'geometry' key in OSRM response!";
        return true;
    }

    // parse geometry into path


    // dummy error return until implemented
    err_msg = "OsrmClient::getGeom() has not been implemented!";
    return true; 
}


