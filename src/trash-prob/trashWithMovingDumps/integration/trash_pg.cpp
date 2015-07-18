#include <iostream>
#include <sstream>
#include <string>
#include <exception>
#include <string.h>

#include "signalhandler.h"
#include "vrptools.h"

#ifdef OSRMCLIENT
#include "osrmclient.h"
#endif

#ifdef DOVRPLOG
#include "logger.h"
#endif

#include "pg_types_vrp.h"

#include "trashprob.h"
#include "truckManyVisitsDump.h"
#include "fleetOpt.h"


class LoadFromFiles {
  private:
    std::vector<container_t> containers;
    std::vector<otherloc_t> otherlocs;
    std::vector<vehicle_t> vehicles;
    std::vector<ttime_t> ttimes;

  public:
    LoadFromFiles( std::string &filePrefix ) {
        load_containers( filePrefix + ".containers.txt" );
        load_otherlocs( filePrefix + ".otherlocs.txt" );
        load_vehicles( filePrefix + ".vehicles.txt" );
        load_ttimes( filePrefix + ".dmatrix-time.txt" );
    };

    container_t *getContainers(unsigned int &container_count) {
        assert( containers.size() );
        container_count = containers.size();
        container_t *pg_containers = (container_t *) malloc( container_count * sizeof(container_t) );
        assert( pg_containers );

        for ( unsigned int i=0; i<container_count; i++ ) {
            pg_containers[i].id      = containers[i].id;
            pg_containers[i].x       = containers[i].x;
            pg_containers[i].y       = containers[i].y;
            pg_containers[i].open    = containers[i].open;
            pg_containers[i].close   = containers[i].close;
            pg_containers[i].service = containers[i].service;
            pg_containers[i].demand  = containers[i].demand;
            pg_containers[i].sid     = containers[i].sid;
        }
        return pg_containers;
    };

    otherloc_t *getOtherlocs(unsigned int &otherloc_count) {
        assert( otherlocs.size() );
        otherloc_count = otherlocs.size();
        otherloc_t *pg_otherlocs = (otherloc_t *) malloc( otherloc_count * sizeof(otherloc_t) );
        assert( pg_otherlocs );

        for ( unsigned int i=0; i<otherloc_count; i++ ) {
            pg_otherlocs[i].id    = otherlocs[i].id;
            pg_otherlocs[i].x     = otherlocs[i].x;
            pg_otherlocs[i].y     = otherlocs[i].y;
            pg_otherlocs[i].open  = otherlocs[i].open;
            pg_otherlocs[i].close = otherlocs[i].close;
        }
        return pg_otherlocs;
    };

    vehicle_t *getVehicles(unsigned int &vehicle_count) {
        assert( vehicles.size() );
        vehicle_count = vehicles.size();
        vehicle_t *pg_vehicles = (vehicle_t *) malloc( vehicle_count * sizeof(vehicle_t) );
        assert( pg_vehicles );

        for ( unsigned int i=0; i<vehicle_count; i++ ) {
            pg_vehicles[i].vid              = vehicles[i].vid;
            pg_vehicles[i].start_id         = vehicles[i].start_id;
            pg_vehicles[i].dump_id          = vehicles[i].dump_id;
            pg_vehicles[i].end_id           = vehicles[i].end_id;
            pg_vehicles[i].capacity         = vehicles[i].capacity;
            pg_vehicles[i].dumpservicetime  = vehicles[i].dumpservicetime;
            pg_vehicles[i].starttime        = vehicles[i].starttime;
            pg_vehicles[i].endtime          = vehicles[i].endtime;
        }
        return pg_vehicles;
    };

    ttime_t *getTtimes(unsigned int &ttime_count) {
        ttime_count = ttimes.size();

        // if not ttime data then return NULL
        if (ttime_count == 0) return (ttime_t *) 0;

        ttime_t *pg_ttimes = (ttime_t *) malloc( ttime_count * sizeof(ttime_t) );
        assert( pg_ttimes );

        for ( unsigned int i=0; i<ttime_count; i++ ) {
            pg_ttimes[i].from_id = ttimes[i].from_id;
            pg_ttimes[i].to_id = ttimes[i].to_id;
            pg_ttimes[i].ttime = ttimes[i].ttime;
        }
        return pg_ttimes;
    };

  private:
    void load_containers( std::string infile ) {
        std::ifstream in( infile.c_str() );
        std::string line;
        containers.clear();
        while ( getline( in, line ) ) {
            if ( line[0] == '#' ) continue;
            container_t container = parseContainer( line );
            containers.push_back( container );
        }
        in.close();
    };

    void load_otherlocs( std::string infile ) {
        std::ifstream in( infile.c_str() );
        std::string line;
        otherlocs.clear();
        while ( getline( in, line ) ) {
            if ( line[0] == '#' ) continue;
            otherloc_t otherloc = parseOtherloc( line );
            otherlocs.push_back( otherloc );
        }
        in.close();
    };

    void load_vehicles( std::string infile ) {
        std::ifstream in( infile.c_str() );
        std::string line;
        vehicles.clear();
        while ( getline( in, line ) ) {
            if ( line[0] == '#' ) continue;
            vehicle_t vehicle = parseVehicle( line );
            vehicles.push_back( vehicle );
        }
        in.close();
    };

    void load_ttimes( std::string infile ) {
        std::ifstream in( infile.c_str() );
        std::string line;
        ttimes.clear();
        while ( getline( in, line ) ) {
            if ( line[0] == '#' ) continue;
            ttime_t ttime = parseTtime( line );
            ttimes.push_back( ttime );
        }
        in.close();
    };

    container_t parseContainer( std::string line ) {
        std::istringstream buffer( line );
        container_t container;
        buffer >> container.id;
        buffer >> container.x;
        buffer >> container.y;
        buffer >> container.open;
        buffer >> container.close;
        buffer >> container.demand;
        buffer >> container.service;
        buffer >> container.sid;
        return container;
    };

    otherloc_t parseOtherloc( std::string line ) {
        std::istringstream buffer( line );
        otherloc_t otherloc;
        buffer >> otherloc.id;
        buffer >> otherloc.x;
        buffer >> otherloc.y;
        buffer >> otherloc.open;
        buffer >> otherloc.close;
        return otherloc;
    };

    vehicle_t parseVehicle( std::string line ) {
        std::istringstream buffer( line );
        vehicle_t vehicle;
        buffer >> vehicle.vid;
        buffer >> vehicle.start_id;
        buffer >> vehicle.dump_id;
        buffer >> vehicle.end_id;
        buffer >> vehicle.dumpservicetime;
        buffer >> vehicle.capacity;
        buffer >> vehicle.starttime;
        buffer >> vehicle.endtime;
        return vehicle;
    };

    ttime_t parseTtime( std::string line ) {
        std::istringstream buffer( line );
        ttime_t ttime;
        buffer >> ttime.from_id;
        buffer >> ttime.to_id;
        buffer >> ttime.ttime;
        return ttime;
    };

};  // end of class LoadFromFiles


// -------------------------------------------------------------------

void Usage()
{
  std::cout << "Usage: trash file (no extension)\n";
}




int main(int argc, char **argv) {

#ifdef DOVRPLOG
  if ( not google::IsGoogleLoggingInitialized() ) {
    FLAGS_log_dir = "./logs/";
    google::InitGoogleLogging( "vrp_trash_collection" );
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::FATAL;
    FLAGS_minloglevel = google::INFO;
    FLAGS_logbufsecs = 0;
  }
#endif

  if (argc < 2) {
    Usage();
    return 1;
  }

  std::string infile = argv[1];

  try {

#ifdef VRPMINTRACE
   DLOG(INFO) << "log file started for: " << infile;
#endif

#ifdef OSRMCLIENT
    osrmi->useOsrm(true);
    bool testResult = osrmi->testOsrmClient(
          -34.905113, -56.157043,
          -34.906807, -56.158463,
          -34.9076,   -56.157028);
   // remove the following comment when testing OSRM only
   // assert(true==false);
#ifdef VRPMINTRACE
    if (testResult)
     DLOG(INFO) << "osrm test passed";
   else
     DLOG(INFO) << "osrm test FAIL";
#endif  // VRPMINTRACE
#endif  // OSRMCLIENT

    // read the data from the input files
    // and make it look like we loaded it from postgresql

    LoadFromFiles loader( infile );
    unsigned int container_count = 0;
    container_t *containers = loader.getContainers( container_count);
    unsigned int otherloc_count = 0;
    otherloc_t *otherlocs = loader.getOtherlocs( otherloc_count );
    unsigned int vehicle_count = 0;
    vehicle_t *vehicles = loader.getVehicles( vehicle_count );
    unsigned int ttime_count = 0;
    ttime_t *ttimes = loader.getTtimes( ttime_count );

    // start the problem the same way we do from postgresql

    int ret = -1;
    unsigned int iteration = 1000;
    unsigned int check = 0;
    vehicle_path_t *result = NULL;
    int result_count;
    char *err_msg_out = NULL;
    char *err_msg = NULL;

    ret = vrp_trash_collection(
          containers, container_count,
          otherlocs, otherloc_count,
          vehicles, vehicle_count,
          ttimes, ttime_count,
          iteration, check,
          &result, &result_count, &err_msg, &err_msg_out );

    std::cout << "return from vrp_trash_collection(), ret=" << ret << std::endl;
    std::cout << "Message received from inside: ";
    if (check) 
        if (err_msg_out) std::cout << err_msg_out;
    else
        if (err_msg) std::cout << err_msg;
    std::cout << std::endl;

    if ( ret >= 0 ) {
        std::cout << "Results:" << std::endl;
        std::cout << "-------------------------------" << std::endl;
        for ( int i=0; i<result_count; i++ ) {
            std::cout << i << "\t"
                      << result[i].seq << "\t"
                      << result[i].vid << "\t"
                      << result[i].nid << "\t"
                      << result[i].ntype << "\t"
                      << result[i].deltatime << "\t"
                      << result[i].cargo << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
    }

    if (containers) free(containers);
    if (otherlocs) free(otherlocs);
    if (vehicles) free(vehicles);
    if (ttimes) free(ttimes);
    if (result) free(result);
    

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  return 0;
}

