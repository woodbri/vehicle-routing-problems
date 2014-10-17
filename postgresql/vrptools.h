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

#include "postgres.h"

// input data model

typedef struct container {
    int id;
    float8 x;
    float8 y;
    int open;
    int close;
    int service;
    int demand;
    int sid;
} container_t;

typedef struct otherloc {
    int id;
    float8 x;
    float8 y;
    int open;
    int close;
} otherloc_t;

typedef struct vehicle {
    int vid;
    int start_id;
    int dump_id;
    int end_id;
    int capacity;
    int dumpservicetime;
    int starttime;
    int endtime;
} vehicle_t;

typedef struct ttime {
    int from_id;
    int to_id;
    float8 ttime;
} ttime_t;

// Output data structure

typedef struct vehicle_path {
    int seq;
    int vid;
    int nid;
    int ntype;
    float8 deltatime;
    float8 cargo;
} vehicle_path_t;


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
