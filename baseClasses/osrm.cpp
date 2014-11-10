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

#include "osrm.h"


bool VrpOSRM::getTravelTime( double &ttime ) const {
    struct json_object *jtree;
    struct json_object *jobj;

    jtree = json_tokener_parse( json.c_str() );

    if ( !jtree ) {
        std::cout << "Error: Invalid json document in OSRM response!" << std::endl;
        return true;
    }

    jobj = json_object_object_get( jtree, "route_summary" );

    if ( !jobj ) {
        std::cout << "Error: Failed to find 'route_summary' key in json document!" <<
                  std::endl;
        json_object_put( jtree );
        return true;
    }

    jobj = json_object_object_get( jobj, "total_time" );

    if ( !jobj ) {
        std::cout << "Error: Failed to find 'total_time' key in json document!" <<
                  std::endl;
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
        std::cout << "Null json document in OSRM response!" << std::endl;
        return true;
    }

    jtree = json_tokener_parse( json.c_str() );

    if ( !jtree ) {
        std::cout << "Error: Invalid json document in OSRM response!" << std::endl;
        return true;
    }

    jobj = json_object_object_get( jtree, "status" );

    if ( !jobj ) {
        json_object_put( jtree );
        std::cout << "Error: Error parsing OSRM response, \"status\" not found." <<
                  std::endl;
        return true;
    }

    status = json_object_get_int( jobj );
    json_object_put( jtree );

    return false;
}


bool VrpOSRM::callOSRM( const std::string url ) {
    std::stringstream result;

    //std::cout << "callOSRM: url: " << url << std::endl;

    try {

        curlpp::Easy request;
        request.setOpt( new cURLpp::Options::Url( url ) );
        request.setOpt( new cURLpp::Options::WriteStream( &result ) );
        request.perform();
        json = result.str();

    }
    catch ( curlpp::LogicError &e ) {
        std::cout << e.what() << std::endl;
        return true;
    }
    catch ( curlpp::RuntimeError &e ) {
        std::cout << e.what() << std::endl;
        return true;
    }

    return false;
}


