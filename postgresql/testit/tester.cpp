
#include <cassert>
#include <stdexcept>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "../vrptools.h"
#include "../trashprob.h"
#include "osrm.h"

#define LOG std::cout

int main(int argc, char **argv) {
    cURLpp::Cleanup myCleanup;

    try {
        int container_count;
        int otherloc_count;
        int vehicle_count;
        int ttime_count;
        container_t *containers;
        otherloc_t *otherlocs;
        vehicle_t *vehicles;
        ttime_t *ttimes;
        
        std::ifstream in( "test.txt" );
        std::string line;

        do {
            getline(in, line);
            std::istringstream buffer( line );
            buffer >> container_count;
            buffer >> otherloc_count;
            buffer >> vehicle_count;
            buffer >> ttime_count;
        } while (false);

        LOG << "Header: " << container_count
            << ", " << otherloc_count
            << ", " << vehicle_count
            << ", " << ttime_count
            << "\n";

        containers = (container_t *) malloc( container_count * sizeof(container_t) );
        otherlocs = (otherloc_t *) malloc( otherloc_count * sizeof(otherloc_t) );
        vehicles = (vehicle_t *) malloc( vehicle_count * sizeof(vehicle_t) );
        ttimes = (ttime_t *) malloc( ttime_count * sizeof(ttime_t) );

        LOG << "reading containers ...\n";
        getline(in, line); LOG << line << "\n";
        assert(line[0] == '-');
            
        for (int i=0; i<container_count; ++i) {
            getline(in, line); LOG << line << "\n";
            std::istringstream buffer( line );
            buffer >> containers[i].id;
            buffer >> containers[i].x;
            buffer >> containers[i].y;
            buffer >> containers[i].open;
            buffer >> containers[i].close;
            buffer >> containers[i].service;
            buffer >> containers[i].demand;
        }

        LOG << "reading otherlocs ...\n";
        getline(in, line); LOG << line << "\n";
        assert(line[0] == '-');
            
        for (int i=0; i<otherloc_count; ++i) {
            getline(in, line); LOG << line << "\n";
            std::istringstream buffer( line );
            buffer >> otherlocs[i].id;
            buffer >> otherlocs[i].x;
            buffer >> otherlocs[i].y;
            buffer >> otherlocs[i].open;
            buffer >> otherlocs[i].close;
        }

        LOG << "reading vehicles ...\n";
        getline(in, line); LOG << line << "\n";
        assert(line[0] == '-');

        for (int i=0; i<vehicle_count; ++i) {
            getline(in, line); LOG << line << "\n";
            std::istringstream buffer( line );
            buffer >> vehicles[i].vid;
            buffer >> vehicles[i].start_id;
            buffer >> vehicles[i].dump_id;
            buffer >> vehicles[i].end_id;
            buffer >> vehicles[i].capacity;
            buffer >> vehicles[i].dumpservicetime;
            buffer >> vehicles[i].starttime;
            buffer >> vehicles[i].endtime;
        }

        LOG << "reading ttimes ...\n";
        getline(in, line); LOG << line << "\n";
        assert(line[0] == '-');

        for (int i=0; i<ttime_count; ++i) {
            getline(in, line);
            std::istringstream buffer( line );
            buffer >> ttimes[i].from_id;
            buffer >> ttimes[i].to_id;
            buffer >> ttimes[i].ttime;
        }
        
        in.close();

        LOG << "data read, continuing ...\n";

        int ret;
        vehicle_path_t *result;
        int result_count;
        char *err_msg;

        ret = vrp_trash_collection( containers, container_count,
                                    otherlocs, otherloc_count,
                                    vehicles, vehicle_count,
                                    ttimes, ttime_count,
                                    &result, &result_count, &err_msg);

        std::cout << "ret: " << ret << std::endl;
        if (err_msg) std::cout << "msg: " << err_msg << std::endl;

        if (result_count > 0) {
            for (int i=0; i<result_count; ++i) {
                std::cout << result[i].seq
                    << "\t" << result[i].vid
                    << "\t" << result[i].nid
                    << "\t" << result[i].ntype
                    << "\t" << result[i].deltatime
                    << "\t" << result[i].cargo
                    << std::endl;
            }
            free(result);
        }

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}

