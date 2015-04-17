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

#include <exception>
#include <string.h>

#include "signalhandler.h"
#include "vrptools.h"

#ifdef DOSTATS
#include "timer.h"
#endif

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef OSRMCLIENT
#include "osrmclient.h"
#endif

#include "trashprob.h"
#include "feasableSolLoop.h"
#include "tabuopt.h"

//#define PGR_LOGGER_ON
#include "./minilog.h"


int vrp_trash_collection( container_t *containers, unsigned int container_count,
                          otherloc_t *otherlocs, unsigned int otherloc_count,
                          vehicle_t *vehicles, unsigned int vehicle_count,
                          ttime_t *ttimes, unsigned int ttime_count,
                          unsigned int iteration, unsigned int check,
                          vehicle_path_t **vehicle_paths, int *vehicle_path_count,
                          char **err_msg, char **data_err_msg )
{

  try {
    // register the signal handler
    REG_SIGINT

#ifdef DOVRPLOG

    if ( not google::IsGoogleLoggingInitialized() ) {
      FLAGS_log_dir = "/tmp/";
      google::InitGoogleLogging( "vrp_trash_collection" );
      FLAGS_logtostderr = 0;
      FLAGS_stderrthreshold = google::FATAL;
      FLAGS_minloglevel = google::INFO;
    }

#endif


    TrashProb prob(containers, container_count, otherlocs, otherloc_count, ttimes,
                   ttime_count, vehicles, vehicle_count) ;


    if (check == 1) {
      if ( prob.isValid() )
        *data_err_msg = strdup( "OK" );
      else
        *data_err_msg = strdup( prob.getErrorsString().c_str() );

      twc->cleanUp();
      return 0;
    }


    if ( not prob.isValid() ) {
      *err_msg = strdup( prob.getErrorsString().c_str() );
      twc->cleanUp();
      return -1;
    }

    THROW_ON_SIGINT

    FeasableSolLoop tp( prob );
    tp.computeCosts();

    THROW_ON_SIGINT

    TabuOpt ts( tp , iteration);

    unsigned long int count = 0;
    *vehicle_paths = ts.getSolutionForPg( count );
    *vehicle_path_count = count;

    if ( count == -1 ) {
      *err_msg = strdup ( "Failed to allocate memory for results!");
      twc->cleanUp();
      return -1;
    }

    twc->cleanUp();
  } catch ( std::exception &e ) {
#ifdef DOVRPLOG
    DLOG(INFO) << "in wrapper, caught exception: " << e.what();
#endif
    *err_msg = strdup( e.what() );
    return -1;
  } catch ( ... ) {
#ifdef DOVRPLOG
    DLOG(INFO) << "in wrapper, caught unknown expection!";
#endif
    *err_msg = strdup( "Caught unknown expection!" );
    return -1;
  }

  *err_msg = (char *)0;


  return EXIT_SUCCESS;
}


int get_osrm_route_geom( float8 *lat, float8 *lon, int num, char **gtext,
          char **err_msg ) {

#ifdef OSRMCLIENT
  bool ret;

  try {
    // register the signal handler
    REG_SIGINT

#ifdef DOVRPLOG

    if ( not google::IsGoogleLoggingInitialized() ) {
      FLAGS_log_dir = "/tmp/";
      google::InitGoogleLogging( "vrp_trash_collection" );
      FLAGS_logtostderr = 0;
      FLAGS_stderrthreshold = google::FATAL;
      FLAGS_minloglevel = google::INFO;
      PGR_LOG("Initializing InitGoogleLogging");
    }

#endif

    DLOG(INFO) << "Called get_osrm_route_geom";
    PGR_LOG("Called get_osrm_route_geom");

    osrmi->useOsrm( true );

    std::string err = osrmi->getErrorMsg();
    PGR_LOG( err.c_str() );

    if (not osrmi->getConnection()) {
#ifdef DOVRPLOG
        DLOG(INFO) << "in wrapper, OSRM connection is not available!";
#endif
        PGR_LOG("in wrapper, OSRM connection is not available!");
        *err_msg = strdup( "OSRM connection is not available!" );
        return -1;
    }

    osrmi->clear();
    osrmi->setWantGeometryText( true );

    for (int i=0; i<num; i++) {
        osrmi->addViaPoint(lat[i], lon[i]);
#ifdef DOVRPLOG
        DLOG(INFO) << i << "\t" << lat[i] << "\t" << lon[i];
#endif
    }

    THROW_ON_SIGINT

    std::string geom;

    if (osrmi->getOsrmViaroute()) {
        // success
        PGR_LOG( osrmi->getHttpContent().c_str() );
        if (osrmi->getOsrmGeometryText( geom )) {
            *gtext = strdup( geom.c_str() );
        }
        else {
#ifdef DOVRPLOG
            DLOG(INFO) << osrmi->getErrorMsg();
            DLOG(INFO) << "in wrapper, failed to extract geometry text!";
#endif
            std::string msg;
            msg = "getOsrmViaroute failed to extract geometry text! : " + osrmi->getErrorMsg();
            PGR_LOG( msg.c_str() );
            *err_msg = strdup( msg.c_str() );
            return -1;
        }
    }
    else {
#ifdef DOVRPLOG
        DLOG(INFO) << "in wrapper, getOsrmViaroute failed to return a route!";
#endif
        *err_msg = strdup( "getOsrmViaroute failed to return a route!" );
        return -1;
    }


  } catch ( std::exception &e ) {
#ifdef DOVRPLOG
    DLOG(INFO) << "in wrapper, caught exception: " << e.what();
#endif
    *err_msg = strdup( e.what() );
    return -1;
  } catch ( ... ) {
#ifdef DOVRPLOG
    DLOG(INFO) << "in wrapper, caught unknown expection!";
#endif
    *err_msg = strdup( "Caught unknown expection!" );
    return -1;
  }

  *err_msg = (char *)0;

  return EXIT_SUCCESS;
#else   // OSRMCLIENT
  *err_msg = strdup( "OSRM was not complied into this extension!" );
  return -1;
#endif  // OSRMCLIENT
}


