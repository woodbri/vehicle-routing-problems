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
#ifndef OSRM_H
#define OSRM_H

#define OSRMCURL_GET 1
#define OSRMCURL_PUT 2
#define OSRMCURL_POST 3
#define OSRMCURL_DELETE 4

// curlpp specific
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

// json-c lib headers
#include <json/json.h>

/*! \class OSRM
 * \brief Simple interface class to OSRM server.
 *
 * The OSRM class functions take a URL with a viaroute request to an OSRM
 * server and and assists with the extraction of the travel time for
 * the requested route.
 */
class OSRM {

  private:
    std::string json;   ///< local storage for the json response string.

  public:

    /*! \fn bool getTravelTime(double& ttime) const
     * \brief Extract the travel time from the request.
     * \return true if there is an error.
     * \param[out] ttime The extracted travel time.
     */
    bool getTravelTime( double &ttime ) const;

    /*! \fn bool bool getStatus(int& status) const
     * \brief Extract the OSRM request status from the request.
     * \return true if there is an error.
     * \param[out] status The extracted OSRM status value.
     */
    bool getStatus( int &status ) const;

    /*! \fn bool callOSRM(const std::string url)
     * \brief Make an HTTP request to the OSRM url.
     * \return true if there is an error.
     * \param[in] url A URL for a viaroute request to an OSRM server.
     */
    bool callOSRM( const std::string url );

};

#endif
