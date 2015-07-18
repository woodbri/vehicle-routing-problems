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
#include "truckManyVisitsDump.h"
#include "fleetOpt.h"

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
    PGR_LOGF("google::IsGoogleLoggingInitialized: %d\n", google::IsGoogleLoggingInitialized());
#endif

    osrmi->useOsrm( true );

    std::string err = osrmi->getErrorMsg();

    if (not osrmi->getConnection()) {
#ifdef DOVRPLOG
        DLOG(INFO) << "in wrapper, OSRM connection is not available!";
#endif
        *err_msg = strdup( "OSRM connection is not available!" );
        return -1;
    }

#ifdef DOVRPLOG
    DLOG(INFO) << "Starting vrp_trash_collection(): num. container: "
               << container_count << ", num. other_loc: "
               << otherloc_count  << ", num. vehicle: "
               << vehicle_count   << ", num. ttime: "
               << ttime_count     << ", num. iteration: "
               << iteration;
#endif



    TrashProb prob(containers, container_count, otherlocs, otherloc_count,
                   ttimes, ttime_count, vehicles, vehicle_count, check);

#if 0
    DLOG(INFO) << "Problem definition -----------------------";
    prob.dumpdataNodes();
#endif

    if (check == 1) {
      if ( prob.isValid() or prob.getErrorsString().size() == 0 )
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

    TruckManyVisitsDump tp( prob );
    tp.process(0);
#ifdef DOVRPLOG
    DLOG(INFO) << "Initial solution: 0 is best";
#endif

    double best_cost = 9999999;
    Solution best_sol( tp );
    best_cost = best_sol.getCostOsrm();

    THROW_ON_SIGINT

#if 0
    best_sol.dumpSolutionForPg();
#endif

#if 0
    TabuOpt tsi( tp , iteration);
    Solution opt_sol = tsi.getBestSolution();

    if (best_cost > opt_sol.getCostOsrm()) {
#ifdef DOVRPLOG
      DLOG(INFO) << "Optimization: 0 is best";
#endif
      best_cost = opt_sol.getCostOsrm();
      best_sol = opt_sol;
    }
#endif

    for (int icase = 1; icase < 7; ++icase) {
#ifdef DOVRPLOG
      DLOG(INFO) << "initial solution: " << icase;
#endif
      tp.process(icase);
      if (best_cost > tp.getCostOsrm()) {
#ifdef DOVRPLOG
        DLOG(INFO) << "initial solution: " << icase << " is best";
#endif
        best_cost = tp.getCostOsrm();
        best_sol = tp;
      }

      THROW_ON_SIGINT

    }

#if 0
    DLOG(INFO) << "Best initial solution selected";
    best_sol.dumpSolutionForPg();
#endif

    Optimizer optSol(best_sol, iteration);
    if (best_cost > optSol.getCostOsrm()) {
        best_cost = optSol.getCostOsrm();
        best_sol = optSol;
    }

#ifdef DOVRPLOG
    DLOG(INFO) << "=-=-=-=-=-=- OPTIMIZED SOLUTION -=-=-=-=-=-=-=";
    DLOG(INFO) << "Number of containers: " << best_sol.countPickups();
    best_sol.dumpCostValues();
    DLOG(INFO) << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=";
    best_sol.tau();
    DLOG(INFO) << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=";
#endif

    unsigned long int count = 0;
    *vehicle_paths = best_sol.getSolutionForPg( count );
    *vehicle_path_count = count;

    twc->cleanUp();

    if ( count == -1 ) {
      *err_msg = strdup ( "Failed to allocate memory for results!");
      twc->cleanUp();
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
}


int get_osrm_route_geom( float8 *lat, float8 *lon, int num, double *time,
          char **gtext, char **err_msg ) {

#ifdef OSRMCLIENT
  bool ret;

  try {
    // register the signal handler
    REG_SIGINT

    DLOG(INFO) << "Called get_osrm_route_geom";
    PGR_LOG("Called get_osrm_route_geom");

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

    osrmi->useOsrm( true );

    std::string err = osrmi->getErrorMsg();
    PGR_LOGF("osrmi err: '%s'\n", err.c_str() );

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
        //PGR_LOGF("%d: %.6f, %.6f\n", i, lat[i], lon[i]);
#endif
    }

    THROW_ON_SIGINT

    std::string geom;
    double otime;

    if (osrmi->getOsrmViaroute()) {
        // success
        PGR_LOG( osrmi->getHttpContent().c_str() );
        if (osrmi->getOsrmGeometryText( geom )) {
            *gtext = strdup( geom.c_str() );
            osrmi->getOsrmTime( otime );
            *time = otime;
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


