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

    rapidjson::Document jsondoc;
    jsondoc.Parse( json.c_str() );

    if ( jsondoc.HasParseError() ) {
        DLOG( INFO ) << "Error: Invalid json document in OSRM response!";
        return true;
    }

    if ( not jsondoc.HasMember( "route_summary" ) ) {
        DLOG( INFO ) << "Error: Failed to find 'route_summary' key in json document!";
        return true;
    }

    if ( not jsondoc["route_summary"].HasMember( "total_time" ) ) {
        DLOG( INFO ) << "Error: Failed to find 'total_time' key in json document!";
        return true;
    }

    ttime = jsondoc["route_summary"]["total_time"].GetDouble() / 60.0;

    return false;
}


bool VrpOSRM::getStatus( int &status ) const {

    status = -1;

    if ( json.size() == 0 ) {
        DLOG( INFO ) << "Null json document in OSRM response!";
        return true;
    }

    rapidjson::Document jsondoc;
    jsondoc.Parse( json.c_str() );

    if ( jsondoc.HasParseError() ) {
        DLOG( INFO ) << "Error: Invalid json document in OSRM response!";
        return true;
    }

    if ( not jsondoc.HasMember( "status" ) ) {
        DLOG( INFO ) << "Error: Failed to find 'status' key in json document!";
        return true;
    }

    status = jsondoc["status"].GetInt();

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
        DLOG( WARNING ) << e.what();
        return true;
    }
    catch ( curlpp::RuntimeError &e ) {
        DLOG( WARNING ) << e.what();
        return true;
    }

    return false;
}


