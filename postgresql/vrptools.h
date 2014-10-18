/*
 *
 * vrptools interface for PostgreSQL
 *
 * Copyright (c) 2014 Stephen Woodbridge
 *
 * Licensed under BSD 2 Clause Open Source License.
 *
 */

#ifndef VRPTOOLS_H
#define VRPTOOLS_H

#include "pg_types_vrp.h"

#ifdef __cplusplus
extern "C"
#endif

int vrp_trash_collection (
        container_t *containers,
        unsigned int container_count,
        otherloc_t *otherlocs,
        unsigned int otherloc_count,
        vehicle_t *vehicles,
        unsigned int vehicle_count,
        ttime_t *ttime,
        unsigned int ttime_count,
        vehicle_path_t **vehicle_paths,
        int *vehicle_path_count,
        char **err_msg
        );

#endif
