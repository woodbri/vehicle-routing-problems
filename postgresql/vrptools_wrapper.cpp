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

#include "vrptools.h"
#include "timer.h"
#include "trashprob.h"
#include "feasableSolLoop.h"
#include "tabusearch.h"

#define LOG std::cout
#define log std

int vrp_trash_collection( container_t *containers, unsigned int container_count,
        otherloc_t *otherlocs, unsigned int otherloc_count,
        vehicle_t *vehicles, unsigned int vehicle_count,
        ttime_t *ttimes, unsigned int ttime_count,
        vehicle_path_t **vehicle_paths, int *vehicle_path_count,
        char **err_msg) {

    try {
        Timer starttime;
        
        TrashProb prob;
        prob.addContainers( containers, container_count );
        prob.addOtherlocs( otherlocs, otherloc_count );

        if (not prob.checkNodesOk()) {
            std::string err = prob.whatIsWrong();
            *err_msg = strdup(err.c_str());
            return -1;
        }

        prob.addTtimes( ttimes, ttime_count );
        prob.addVehicles( vehicles, vehicle_count );

        if (not prob.isValid()) {
            std::string err = prob.whatIsWrong();
            *err_msg = strdup(err.c_str());
            return -1;
        }

        FeasableSolLoop tp(prob);
        tp.computeCosts();

        LOG << "Load and initial solution time: " << starttime.duration()
            << ", initial cost: " << tp.getCost()
            << log::endl;

        Timer searchtime;

        TabuSearch ts(tp);
        ts.setMaxIteration(1000);
        ts.search();

        Solution best = ts.getBestSolution();
        best.computeCosts();

        LOG << "Tabu search time: " << searchtime.duration()
            << ", final cost: " << best.getCost()
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
