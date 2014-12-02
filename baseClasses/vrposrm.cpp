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

#include <iostream>
#include <sstream>
#include <string>

#include "logger.h"
#include "vrposrm.h"


bool VrpOSRM::getTravelTime( double &ttime ) const {
    struct json_object *jtree;
    struct json_object *jobj;

    jtree = json_tokener_parse( json.c_str() );

    if ( !jtree ) {
        DLOG(INFO) << "Error: Invalid json document in OSRM response!" << std::endl;
        return true;
    }

    jobj = json_object_object_get( jtree, "route_summary" );

    if ( !jobj ) {
        DLOG(INFO) << "Error: Failed to find 'route_summary' key in json document!";
        json_object_put( jtree );
        return true;
    }

    jobj = json_object_object_get( jobj, "total_time" );

    if ( !jobj ) {
        DLOG(INFO) << "Error: Failed to find 'total_time' key in json document!";
        json_object_put( jtree );
        return true;
    }

    ttime = ( double ) json_object_get_int( jobj ) / 60.0;
    json_object_put( jtree );

    return false;
}


bool VrpOSRM::getStatus( int &status ) const {
    struct json_object *jtree;
    struct json_object *jobj;

    status = -1;

    if ( json.size() == 0 ) {
        DLOG(INFO) << "Null json document in OSRM response!";
        return true;
    }

    jtree = json_tokener_parse( json.c_str() );

    if ( !jtree ) {
        DLOG(INFO) << "Error: Invalid json document in OSRM response!";
        return true;
    }

    jobj = json_object_object_get( jtree, "status" );

    if ( !jobj ) {
        json_object_put( jtree );
        DLOG(INFO) << "Error: Error parsing OSRM response, \"status\" not found.";
        return true;
    }

    status = json_object_get_int( jobj );
    json_object_put( jtree );

    return false;
}


bool VrpOSRM::callOSRM( const std::string url ) {
    std::stringstream result;

    //DLOG(INFO) << "callOSRM: url: " << url;

    try {

        curlpp::Easy request;
        request.setOpt( new cURLpp::Options::Url( url ) );
        request.setOpt( new cURLpp::Options::WriteStream( &result ) );
        request.perform();
        json = result.str();

    }
    catch ( curlpp::LogicError &e ) {
        DLOG(WARNING) << e.what();
        return true;
    }
    catch ( curlpp::RuntimeError &e ) {
        DLOG(WARNING) << e.what();
        return true;
    }

    return false;
}


