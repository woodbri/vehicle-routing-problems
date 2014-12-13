-- complain if script is sourced in psql, rather than via CREATE EXTENSION
\echo Use "CREATE EXTENSION vrptools" to load this file. \quit

---------------------------------------------------------------------
-- Core functions to access vrptools from postgresql
-- Authoe: Stephen Woodbridge <woodbri@swoodbridge.com>
-- Date: 2014-10-17
---------------------------------------------------------------------

CREATE OR REPLACE FUNCTION vrp_trashCollection(
        IN container_sql text,
        IN otherloc_sql text,
        IN vehicle_sql text,
        IN ttime_sql text,
        IN interation integer default 1000,
        OUT seq integer,
        OUT vehicle_id integer,
        OUT node_id integer,
        OUT node_type integer,
        OUT delta_time float8,
        OUT cargo float8
    ) RETURNS RECORD
    AS 'MODULE_PATHNAME', 'vrp_trash_collection_run'
    LANGUAGE c STABLE STRICT;

