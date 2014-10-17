
#include <exception>

#include "vrptools.h"
#include "timer.h"
#include "prob_trash.h"
#include "feasableSolLoop.h"
#include "tabusearch.h"

int vrp_trash_collection( container_t *containers, unsigned int container_count,
        otherloc_t *otherlocs, unsigned int otherloc_count,
        vehicle_t *vehicles, unsigned int vehicle_count,
        ttime_t *ttimes. unsignrf int ttime_count,
        vehicle_path_t **vehicle_paths, int *vehicle_path_count,
        char **err_msg) {

    try {
        Timer starttime;
        
        TrashProb prob;
        prob.addContainers( containers, container_count );
        prob.addOtherlocs( otherlocs, otherloc_count );
        prob.addVehicles( vehicles, vehicle_count );
        prob.addTtimes( ttimes, ttime_count );

        FeasableSolLoop tp(prob);
        tp.computeCosts();

        LOG << "Load and initial solution time: " << starttime.duration()
            << ", initial cost: " << tp.getCosts()
            << log::endl;

        Timer searchtime;

        TabuSearch ts(tp);
        ts.setMaxIteraction(1000);
        ts.search();

        Solution best = ts.getBestSolution();
        best.computeCosts();

        LOG << "Tabu search time: " << searchtime.duration()
            << ", final cost: " << best.getCosts()
            << log::endl;

        int count = 0;
        *vehicle_paths = best.getSolutionForPg(count);
        *vehicle_path_count = count;

        if (count == -1) {
            *err_msg = (char *) "Failed to allocate memory for results!";
            return -1;
        }
    }
    catch(std::exception& e) {
        *err_msg = (char *) e.what();
        return -1;
    }
    catch(...) {
        *err_msg = (char *) "Caught unknown expection!";
        return -1;
    }

    return EXIT_SUCCESS;
}
