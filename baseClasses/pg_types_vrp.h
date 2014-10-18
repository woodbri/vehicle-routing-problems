#ifndef PG_TYPES_VRP_H
#define PG_TYPES_VRP_H

// input data model

typedef struct container_ {
    int id;
    double x;
    double y;
    int open;
    int close;
    int service;
    int demand;
    int sid;
} container_t;

typedef struct otherloc_ {
    int id;
    double x;
    double y;
    int open;
    int close;
} otherloc_t;

typedef struct vehicle_ {
    int vid;
    int start_id;
    int dump_id;
    int end_id;
    int capacity;
    int dumpservicetime;
    int starttime;
    int endtime;
} vehicle_t;

typedef struct ttime_ {
    int from_id;
    int to_id;
    double ttime;
} ttime_t;

// Output data structure

typedef struct vehicle_path_ {
    int seq;
    int vid;
    int nid;
    int ntype;
    double deltatime;
    double cargo;
} vehicle_path_t;

#endif
