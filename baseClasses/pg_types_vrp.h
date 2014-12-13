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
#ifndef PG_TYPES_VRP_H
#define PG_TYPES_VRP_H

/*! \file pg_types_vrp.h
 * \brief Defines the structure types for integration with postgresql
 *
 * This file contains the type definitions for integrating the code with
 * postgresql. The interface with postgresql consists of passing data via
 * structures containing containers, other locations, vehicles, and a travel
 * time matrix, then returning results via a vehicle path structure.
 *
 * The structures defined in this file define the data structures being passed.
 */

// input data model

typedef struct container_ {
    int id;
    double x;
    double y;
    double open;
    double close;
    double service;
    double demand;
    int sid;
} container_t;

typedef struct otherloc_ {
    int id;
    double x;
    double y;
    double open;
    double close;
} otherloc_t;

typedef struct vehicle_ {
    int vid;
    int start_id;
    int dump_id;
    int end_id;
    double capacity;
    double dumpservicetime;
    double starttime;
    double endtime;
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
