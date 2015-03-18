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
  unsigned int iteration,
  unsigned int check,
  vehicle_path_t **vehicle_paths,
  int *vehicle_path_count,
  char **err_msg,
  char **data_err_msg
);

#ifdef __cplusplus
extern "C"
#endif

int get_osrm_route_geom(
  float8 *lat,
  float8 *lon,
  int num,
  char **gtext,
  char **err_msg
);
#endif
