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

class OSRM {

  private:
    std::string json;

  public:

    bool getTravelTime(double& ttime) const;
    bool getStatus(int& status) const;

    bool callOSRM(const std::string url);

};

#endif
