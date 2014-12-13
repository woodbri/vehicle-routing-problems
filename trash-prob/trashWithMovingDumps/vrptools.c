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

#include "pg_config.h"
#include "postgres.h"
#include "executor/spi.h"
#include "funcapi.h"
#include "catalog/pg_type.h"

#if PG_VERSION_NUM/100 > 902
#include "access/htup_details.h"
#endif

#include "fmgr.h"
#include "vrptools.h"

#ifdef PG_MODULE_MAGIC
PG_MODULE_MAGIC;
#endif

Datum vrp_trash_collection_run( PG_FUNCTION_ARGS );

#undef DEBUG
#define DEBUG 1

#ifdef DEBUG
#include <stdio.h>
#define DBG(format, arg...)                     \
    elog(NOTICE, format , ## arg)
#else
#define DBG(format, arg...) do { ; } while (0)
#endif

// The number of tuples to fetch from the SPI cursor at each iteration
#define TUPLIMIT 1000

typedef struct container_columns {
    int id;
    int x;
    int y;
    int open;
    int close;
    int service;
    int demand;
    int sid;
} container_columns_t;

typedef struct otherloc_columns {
    int id;
    int x;
    int y;
    int open;
    int close;
} otherloc_columns_t;

typedef struct vehicle_columns {
    int vid;
    int start_id;
    int dump_id;
    int end_id;
    int capacity;
    int dumpservicetime;
    int starttime;
    int endtime;
} vehicle_columns_t;

typedef struct ttime_columns {
    int from_id;
    int to_id;
    int ttime;
} ttime_columns_t;


static int finish( int code, int ret ) {
    DBG( "In finish, trying to disconnect from spi %d", ret );
    code = SPI_finish();

    if ( code  != SPI_OK_FINISH ) {
        elog( ERROR, "couldn't disconnect from SPI" );
        return -1 ;
    }
    DBG( "In finish, disconnect from spi %d successfull", ret );

    return ret;
}

static char *text2char( text *in ) {
    char *out = ( char * )palloc( VARSIZE( in ) );

    memcpy( out, VARDATA( in ), VARSIZE( in ) - VARHDRSZ );
    out[VARSIZE( in ) - VARHDRSZ] = '\0';
    return out;
}

/*
 * This function fetches the container columns from the SPITupleTable..
 *
*/

static int fetch_container_columns( SPITupleTable *tuptable,
                                    container_columns_t *container_columns ) {
    container_columns->id       = SPI_fnumber( SPI_tuptable->tupdesc, "id" );
    container_columns->x        = SPI_fnumber( SPI_tuptable->tupdesc, "x" );
    container_columns->y        = SPI_fnumber( SPI_tuptable->tupdesc, "y" );
    container_columns->open     = SPI_fnumber( SPI_tuptable->tupdesc, "open" );
    container_columns->close    = SPI_fnumber( SPI_tuptable->tupdesc, "close" );
    container_columns->service  = SPI_fnumber( SPI_tuptable->tupdesc, "service" );
    container_columns->demand   = SPI_fnumber( SPI_tuptable->tupdesc, "demand" );
    container_columns->sid      = SPI_fnumber( SPI_tuptable->tupdesc, "street_id" );

    if (    container_columns->id       == SPI_ERROR_NOATTRIBUTE
            || container_columns->x        == SPI_ERROR_NOATTRIBUTE
            || container_columns->y        == SPI_ERROR_NOATTRIBUTE
            || container_columns->open     == SPI_ERROR_NOATTRIBUTE
            || container_columns->close    == SPI_ERROR_NOATTRIBUTE
            || container_columns->service  == SPI_ERROR_NOATTRIBUTE
            || container_columns->demand   == SPI_ERROR_NOATTRIBUTE
            || container_columns->sid      == SPI_ERROR_NOATTRIBUTE
       ) {
        elog( ERROR, "Error: container query must return columns "
              "'id', 'x', 'y', 'open', 'close', 'service', 'demand'"
              ", street_id'"
            );
        return -1;
    }

    if (    SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->x )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->y )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->open )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->close )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->service )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->demand )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, container_columns->sid )
            != INT4OID
       ) {
        elog( ERROR, "Error, container column types must be: int4 id"
              ", float8 x, float8 y, float8 open, float8 close"
              ", float8 service, float8 demand"
              ", int4 street_id"
            );
        return -1;
    }

    return 0;
}


/*
 * This function fetches the otherloc columns from the SPITupleTable..
 *
*/

static int fetch_otherloc_columns( SPITupleTable *tuptable,
                                   otherloc_columns_t *otherloc_columns ) {
    otherloc_columns->id       = SPI_fnumber( SPI_tuptable->tupdesc, "id" );
    otherloc_columns->x        = SPI_fnumber( SPI_tuptable->tupdesc, "x" );
    otherloc_columns->y        = SPI_fnumber( SPI_tuptable->tupdesc, "y" );
    otherloc_columns->open     = SPI_fnumber( SPI_tuptable->tupdesc, "open" );
    otherloc_columns->close    = SPI_fnumber( SPI_tuptable->tupdesc, "close" );

    if (    otherloc_columns->id       == SPI_ERROR_NOATTRIBUTE
            || otherloc_columns->x        == SPI_ERROR_NOATTRIBUTE
            || otherloc_columns->y        == SPI_ERROR_NOATTRIBUTE
            || otherloc_columns->open     == SPI_ERROR_NOATTRIBUTE
            || otherloc_columns->close    == SPI_ERROR_NOATTRIBUTE
       ) {
        elog( ERROR, "Error: otherloc query must return columns "
              "'id', 'x', 'y', 'open', 'close'"
            );
        return -1;
    }

    if (    SPI_gettypeid( SPI_tuptable->tupdesc, otherloc_columns->id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, otherloc_columns->x )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, otherloc_columns->y )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, otherloc_columns->open )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, otherloc_columns->close )
            != FLOAT8OID
       ) {
        elog( ERROR, "Error, otherloc column types must be: int4 id, "
              "float8 x, float8 y, float8 open, float8 close, "
            );
        return -1;
    }

    return 0;
}


/*
 * This function fetches the vehicle columns from the SPITupleTable..
 *
*/

static int fetch_vehicle_columns( SPITupleTable *tuptable,
                                  vehicle_columns_t *vehicle_columns ) {
    vehicle_columns->vid      = SPI_fnumber( SPI_tuptable->tupdesc, "vid" );
    vehicle_columns->start_id = SPI_fnumber( SPI_tuptable->tupdesc, "start_id" );
    vehicle_columns->dump_id  = SPI_fnumber( SPI_tuptable->tupdesc, "dump_id" );
    vehicle_columns->end_id   = SPI_fnumber( SPI_tuptable->tupdesc, "end_id" );
    vehicle_columns->capacity = SPI_fnumber( SPI_tuptable->tupdesc, "capacity" );
    vehicle_columns->dumpservicetime = SPI_fnumber( SPI_tuptable->tupdesc,
                                       "dumpservicetime" );
    vehicle_columns->starttime = SPI_fnumber( SPI_tuptable->tupdesc, "starttime" );
    vehicle_columns->endtime  = SPI_fnumber( SPI_tuptable->tupdesc, "endtime" );

    if (    vehicle_columns->vid      == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->start_id == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->dump_id  == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->end_id   == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->capacity == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->dumpservicetime == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->starttime == SPI_ERROR_NOATTRIBUTE
            || vehicle_columns->endtime  == SPI_ERROR_NOATTRIBUTE
       ) {
        elog( ERROR, "Error: vehicle query must return columns "
              "'vid', 'start_id', 'dump_id', 'end_id', 'capacity'"
              ", 'dumpservicetime', 'starttime', 'endtime'"
            );
        return -1;
    }

    if (    SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->vid )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->start_id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->dump_id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->end_id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->capacity )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->dumpservicetime )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->starttime )
            != FLOAT8OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, vehicle_columns->endtime )
            != FLOAT8OID
       ) {
        elog( ERROR, "Error, vehicle column types must be: int4 vid, "
              "int4 start_id, int4 dump_id, int4 end_id, float8 capacity, "
              "float8 dumpservicetime, float8 starttime, float8 endtime"
            );
        return -1;
    }

    return 0;
}


/*
 * This function fetches the traveltime columns from the SPITupleTable..
 *
*/

static int fetch_ttime_columns( SPITupleTable *tuptable,
                                ttime_columns_t *ttime_columns ) {
    ttime_columns->from_id  = SPI_fnumber( SPI_tuptable->tupdesc, "from_id" );
    ttime_columns->to_id    = SPI_fnumber( SPI_tuptable->tupdesc, "to_id" );
    ttime_columns->ttime    = SPI_fnumber( SPI_tuptable->tupdesc, "ttime" );

    if (    ttime_columns->from_id  == SPI_ERROR_NOATTRIBUTE
            || ttime_columns->to_id    == SPI_ERROR_NOATTRIBUTE
            || ttime_columns->ttime    == SPI_ERROR_NOATTRIBUTE
       ) {
        elog( ERROR, "Error: ttime query must return columns "
              "'from_id', 'to_id', 'ttime'"
            );
        return -1;
    }

    if (    SPI_gettypeid( SPI_tuptable->tupdesc, ttime_columns->from_id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, ttime_columns->to_id )
            != INT4OID
            || SPI_gettypeid( SPI_tuptable->tupdesc, ttime_columns->ttime )
            != FLOAT8OID
       ) {
        elog( ERROR, "Error, traveltime column types must be: int4 from_id, "
              "int4 to_id, float8 ttime "
            );
        return -1;
    }

    return 0;
}


/*
 * Fetch a container from a Tuple
 *
*/

static void fetch_container( HeapTuple *tuple, TupleDesc *tupdesc,
                             container_columns_t *columns, container_t *data ) {
    Datum binval;
    bool isnull;

    binval = SPI_getbinval( *tuple, *tupdesc, columns->id, &isnull );

    if ( isnull ) elog( ERROR, "container.id contains a null value" );

    data->id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->x, &isnull );

    if ( isnull ) elog( ERROR, "container.x contains a null value" );

    data->x = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->y, &isnull );

    if ( isnull ) elog( ERROR, "container.y contains a null value" );

    data->y = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->open, &isnull );

    if ( isnull ) elog( ERROR, "container.open contains a null value" );

    data->open = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->close, &isnull );

    if ( isnull ) elog( ERROR, "container.close contains a null value" );

    data->close = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->service, &isnull );

    if ( isnull ) elog( ERROR, "container.service contains a null value" );

    data->service = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->demand, &isnull );

    if ( isnull ) elog( ERROR, "container.demand contains a null value" );

    data->demand = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->sid, &isnull );

    if ( isnull ) elog( ERROR, "container.street_id contains a null value" );

    data->sid = DatumGetInt32( binval );
}


/*
 * Fetch an otherloc from a Tuple
 *
*/

static void fetch_otherloc( HeapTuple *tuple, TupleDesc *tupdesc,
                            otherloc_columns_t *columns, otherloc_t *data ) {
    Datum binval;
    bool isnull;

    binval = SPI_getbinval( *tuple, *tupdesc, columns->id, &isnull );

    if ( isnull ) elog( ERROR, "otherloc.id contains a null value" );

    data->id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->x, &isnull );

    if ( isnull ) elog( ERROR, "otherloc.x contains a null value" );

    data->x = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->y, &isnull );

    if ( isnull ) elog( ERROR, "otherloc.y contains a null value" );

    data->y = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->open, &isnull );

    if ( isnull ) elog( ERROR, "otherloc.open contains a null value" );

    data->open = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->close, &isnull );

    if ( isnull ) elog( ERROR, "otherloc.close contains a null value" );

    data->close = DatumGetFloat8( binval );
}


/*
 * Fetch a vehicle from a Tuple
 *
*/

static void fetch_vehicle( HeapTuple *tuple, TupleDesc *tupdesc,
                           vehicle_columns_t *columns, vehicle_t *data ) {
    Datum binval;
    bool isnull;

    binval = SPI_getbinval( *tuple, *tupdesc, columns->vid, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.vid contains a null value" );

    data->vid = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->start_id, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.start_id contains a null value" );

    data->start_id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->dump_id, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.dump_id contains a null value" );

    data->dump_id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->end_id, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.end_id contains a null value" );

    data->end_id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->capacity, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.capacity contains a null value" );

    data->capacity = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->dumpservicetime, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.dumpservicetime contains a null value" );

    data->dumpservicetime = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->starttime, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.starttime contains a null value" );

    data->starttime = DatumGetFloat8( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->endtime, &isnull );

    if ( isnull ) elog( ERROR, "vehicle.endtime contains a null value" );

    data->endtime = DatumGetFloat8( binval );
}


/*
 * Fetch a vehicle from a Tuple
 *
*/

static void fetch_ttime( HeapTuple *tuple, TupleDesc *tupdesc,
                         ttime_columns_t *columns, ttime_t *data ) {
    Datum binval;
    bool isnull;

    binval = SPI_getbinval( *tuple, *tupdesc, columns->from_id, &isnull );

    if ( isnull ) elog( ERROR, "ttime.from_id contains a null value" );

    data->from_id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->to_id, &isnull );

    if ( isnull ) elog( ERROR, "ttime.to_id contains a null value" );

    data->to_id = DatumGetInt32( binval );

    binval = SPI_getbinval( *tuple, *tupdesc, columns->ttime, &isnull );

    if ( isnull ) elog( ERROR, "ttime.ttime contains a null value" );

    data->ttime = DatumGetFloat8( binval );
}


static int solve_trash_collection(
    char *container_sql,
    char *otherloc_sql,
    char *vehicle_sql,
    char *ttime_sql,
    unsigned int iteration,
    vehicle_path_t **result,
    int *result_count ) {

    int SPIcode;
    SPIPlanPtr SPIplan;
    Portal SPIportal;
    bool moredata = TRUE;
    int ntuples;

    container_t *containers = NULL;
    int container_count = 0;
    container_columns_t container_columns = {
        .id = -1, .x = -1, .y = -1, .open = -1, .close = -1,
        .service = -1, .demand = -1, .sid = -1
    };

    otherloc_t *otherlocs = NULL;
    int otherloc_count = 0;
    otherloc_columns_t otherloc_columns = {
        .id = -1, .x = -1, .y = -1, .open = -1, .close = -1
    };

    vehicle_t *vehicles = NULL;
    int vehicle_count = 0;
    vehicle_columns_t vehicle_columns = {
        .vid = -1, .start_id = -1, .dump_id = -1, .end_id = -1,
        .capacity = -1, .dumpservicetime = -1, .starttime = -1, .endtime = -1
    };

    ttime_t *ttimes = NULL;
    int ttime_count = 0;
    ttime_columns_t ttime_columns = {
        .from_id = -1, .to_id = -1, .ttime = -1
    };

    char *err_msg;
    int ret = -1;

    DBG( "Enter solve_trash_collection\n" );

    SPIcode = SPI_connect();

    if ( SPIcode  != SPI_OK_CONNECT ) {
        elog( ERROR, "solve_trash_collection: couldn't open a connection to SPI" );
        return -1;
    }

    DBG( "Fetching container tuples\n" );

    SPIplan = SPI_prepare( container_sql, 0, NULL );

    if ( SPIplan  == NULL ) {
        elog( ERROR,
              "solve_trash_collection: couldn't create query plan for cpntainers via SPI" );
        return -1;
    }

    if ( ( SPIportal = SPI_cursor_open( NULL, SPIplan, NULL, NULL,
                                        true ) ) == NULL ) {
        elog( ERROR, "solve_trash_collection: SPI_cursor_open('%s') returns NULL",
              container_sql );
        return -1;
    }

    while ( moredata == TRUE ) {
        //DBG("calling SPI_cursor_fetch");
        SPI_cursor_fetch( SPIportal, TRUE, TUPLIMIT );

        if ( SPI_tuptable == NULL ) {
            elog( ERROR, "SPI_tuptable is NULL" );
            return finish( SPIcode, -1 );
        }

        if ( container_columns.id == -1 ) {
            if ( fetch_container_columns( SPI_tuptable, &container_columns ) == -1 )
                return finish( SPIcode, ret );
        }

        ntuples = SPI_processed;

        container_count += ntuples;

        if ( ntuples > 0 ) {
            int t;
            SPITupleTable *tuptable = SPI_tuptable;
            TupleDesc tupdesc = SPI_tuptable->tupdesc;

            if ( !containers )
                containers = palloc( container_count * sizeof( container_t ) );
            else
                containers = repalloc( containers, container_count * sizeof( container_t ) );

            if ( containers == NULL ) {
                elog( ERROR, "Out of memory" );
                return finish( SPIcode, ret );
            }

            for ( t = 0; t < ntuples; t++ ) {
                HeapTuple tuple = tuptable->vals[t];
                fetch_container( &tuple, &tupdesc, &container_columns,
                                 &containers[container_count - ntuples + t] );
            }

            SPI_freetuptable( tuptable );
        }
        else {
            moredata = FALSE;
        }
    }

    SPI_cursor_close( SPIportal );

    /***********************************************************************/

    DBG( "Fetching otherloc tuples\n" );

    SPIplan = SPI_prepare( otherloc_sql, 0, NULL );

    if ( SPIplan  == NULL ) {
        elog( ERROR,
              "solve_trash_collection: couldn't create query plan for otherlocs via SPI" );
        return -1;
    }

    if ( ( SPIportal = SPI_cursor_open( NULL, SPIplan, NULL, NULL,
                                        true ) ) == NULL ) {
        elog( ERROR, "solve_trash_collection: SPI_cursor_open('%s') returns NULL",
              otherloc_sql );
        return -1;
    }

    moredata = TRUE;

    while ( moredata == TRUE ) {
        //DBG("calling SPI_cursor_fetch");
        SPI_cursor_fetch( SPIportal, TRUE, TUPLIMIT );

        if ( SPI_tuptable == NULL ) {
            elog( ERROR, "SPI_tuptable is NULL" );
            return finish( SPIcode, -1 );
        }

        if ( otherloc_columns.id == -1 ) {
            if ( fetch_otherloc_columns( SPI_tuptable, &otherloc_columns ) == -1 )
                return finish( SPIcode, ret );
        }

        ntuples = SPI_processed;

        otherloc_count += ntuples;

        if ( ntuples > 0 ) {
            int t;
            SPITupleTable *tuptable = SPI_tuptable;
            TupleDesc tupdesc = SPI_tuptable->tupdesc;

            if ( !otherlocs )
                otherlocs = palloc( otherloc_count * sizeof( otherloc_t ) );
            else
                otherlocs = repalloc( otherlocs, otherloc_count * sizeof( otherloc_t ) );

            if ( otherlocs == NULL ) {
                elog( ERROR, "Out of memory" );
                return finish( SPIcode, ret );
            }

            for ( t = 0; t < ntuples; t++ ) {
                HeapTuple tuple = tuptable->vals[t];
                fetch_otherloc( &tuple, &tupdesc, &otherloc_columns,
                                &otherlocs[otherloc_count - ntuples + t] );
            }

            SPI_freetuptable( tuptable );
        }
        else {
            moredata = FALSE;
        }
    }

    SPI_cursor_close( SPIportal );

    /***********************************************************************/

    DBG( "Fetching vehicle tuples\n" );

    SPIplan = SPI_prepare( vehicle_sql, 0, NULL );

    if ( SPIplan  == NULL ) {
        elog( ERROR,
              "solve_trash_collection: couldn't create query plan for vehicles via SPI" );
        return -1;
    }

    if ( ( SPIportal = SPI_cursor_open( NULL, SPIplan, NULL, NULL,
                                        true ) ) == NULL ) {
        elog( ERROR, "solve_trash_collection: SPI_cursor_open('%s') returns NULL",
              vehicle_sql );
        return -1;
    }

    moredata = TRUE;

    while ( moredata == TRUE ) {
        //DBG("calling SPI_cursor_fetch");
        SPI_cursor_fetch( SPIportal, TRUE, TUPLIMIT );

        if ( SPI_tuptable == NULL ) {
            elog( ERROR, "SPI_tuptable is NULL" );
            return finish( SPIcode, -1 );
        }

        if ( vehicle_columns.vid == -1 ) {
            if ( fetch_vehicle_columns( SPI_tuptable, &vehicle_columns ) == -1 )
                return finish( SPIcode, ret );
        }

        ntuples = SPI_processed;

        vehicle_count += ntuples;

        if ( ntuples > 0 ) {
            int t;
            SPITupleTable *tuptable = SPI_tuptable;
            TupleDesc tupdesc = SPI_tuptable->tupdesc;

            if ( !vehicles )
                vehicles = palloc( vehicle_count * sizeof( vehicle_t ) );
            else
                vehicles = repalloc( vehicles, vehicle_count * sizeof( vehicle_t ) );

            if ( vehicles == NULL ) {
                elog( ERROR, "Out of memory" );
                return finish( SPIcode, ret );
            }

            for ( t = 0; t < ntuples; t++ ) {
                HeapTuple tuple = tuptable->vals[t];
                fetch_vehicle( &tuple, &tupdesc, &vehicle_columns,
                               &vehicles[vehicle_count - ntuples + t] );
            }

            SPI_freetuptable( tuptable );
        }
        else {
            moredata = FALSE;
        }
    }

    SPI_cursor_close( SPIportal );

    /***********************************************************************/

    DBG( "Fetching ttime tuples\n" );

    SPIplan = SPI_prepare( ttime_sql, 0, NULL );

    if ( SPIplan  == NULL ) {
        elog( ERROR,
              "solve_trash_collection: couldn't create query plan for ttimes via SPI" );
        return -1;
    }

    if ( ( SPIportal = SPI_cursor_open( NULL, SPIplan, NULL, NULL,
                                        true ) ) == NULL ) {
        elog( ERROR, "solve_trash_collection: SPI_cursor_open('%s') returns NULL",
              ttime_sql );
        return -1;
    }

    moredata = TRUE;

    while ( moredata == TRUE ) {
        //DBG("calling SPI_cursor_fetch");
        SPI_cursor_fetch( SPIportal, TRUE, TUPLIMIT );

        if ( SPI_tuptable == NULL ) {
            elog( ERROR, "SPI_tuptable is NULL" );
            return finish( SPIcode, -1 );
        }

        if ( ttime_columns.from_id == -1 ) {
            if ( fetch_ttime_columns( SPI_tuptable, &ttime_columns ) == -1 )
                return finish( SPIcode, ret );
        }

        ntuples = SPI_processed;

        ttime_count += ntuples;

        if ( ntuples > 0 ) {
            int t;
            SPITupleTable *tuptable = SPI_tuptable;
            TupleDesc tupdesc = SPI_tuptable->tupdesc;

            if ( !ttimes )
                ttimes = palloc( ttime_count * sizeof( ttime_t ) );
            else
                ttimes = repalloc( ttimes, ttime_count * sizeof( ttime_t ) );

            if ( ttimes == NULL ) {
                elog( ERROR, "Out of memory" );
                return finish( SPIcode, ret );
            }

            for ( t = 0; t < ntuples; t++ ) {
                HeapTuple tuple = tuptable->vals[t];
                fetch_ttime( &tuple, &tupdesc, &ttime_columns,
                             &ttimes[ttime_count - ntuples + t] );
            }

            SPI_freetuptable( tuptable );
        }
        else {
            moredata = FALSE;
        }
    }

    SPI_cursor_close( SPIportal );

    /*********************************************************************
        We finally have loaded all the data via the SQL and have it in
        structs so call the C++ wrapper and solve the problem.
    **********************************************************************/

    DBG( "Calling solve_trash_collection_wrapper\n" );

    #if 0
    FILE *fh = fopen( "/tmp/test.txt", "wb" );
    int i;

    if ( !fh ) return -1; //a notice of why we are returning????

    fprintf( fh, "%d %d %d %d\n",
             container_count, otherloc_count, vehicle_count, ttime_count );
    fprintf( fh, "------ containers -----\n" );

    for ( i = 0; i < container_count; i++ ) {
        container_t c = containers[i];
        fprintf( fh, "%d %.6lf %.6lf %d %d %d %d\n",
                 c.id, c.x, c.y, c.open, c.close, c.service, c.demand );
    }

    fprintf( fh, "------ otherlocs -----\n" );

    for ( i = 0; i < otherloc_count; i++ ) {
        otherloc_t c = otherlocs[i];
        fprintf( fh, "%d %.6lf %.6lf %d %d\n",
                 c.id, c.x, c.y, c.open, c.close );
    }

    fprintf( fh, "------ vehicles -----\n" );

    for ( i = 0; i < vehicle_count; i++ ) {
        vehicle_t c = vehicles[i];
        fprintf( fh, "%d %d %d %d %d %d %d %d\n",
                 c.vid, c.start_id, c.dump_id, c.end_id, c.capacity,
                 c.dumpservicetime, c.starttime, c.endtime );
    }

    fprintf( fh, "------ ttimes -----\n" );

    for ( i = 0; i < ttime_count; i++ ) {
        ttime_t c = ttimes[i];
        fprintf( fh, "%d %d %.6lf\n", c.from_id, c.to_id, c.ttime );
    }

    fclose( fh );

    #else
    ret = vrp_trash_collection(
              containers, container_count,
              otherlocs, otherloc_count,
              vehicles, vehicle_count,
              ttimes, ttime_count,
              iteration,
              result, result_count, &err_msg );
    #endif

    DBG( "Message received from inside:" );
    DBG( "%s", err_msg );
    DBG( "ret = %i\n", ret );
    DBG( "result_count = %i\n", *result_count );

    if ( ret < 0 ) {
        ereport( ERROR, ( errcode( ERRCODE_E_R_E_CONTAINING_SQL_NOT_PERMITTED ),
                          errmsg( "Error computing solution: %s", err_msg ) ) );
    }

    return finish( SPIcode, ret );
}


PG_FUNCTION_INFO_V1( vrp_trash_collection_run );
Datum vrp_trash_collection_run( PG_FUNCTION_ARGS ) {

    FuncCallContext     *funcctx;
    int                  call_cntr;
    int                  max_calls;
    TupleDesc            tuple_desc;
    vehicle_path_t      *result;
    int                  ret;

    // stuff done only on the first call of the function
    if ( SRF_IS_FIRSTCALL() ) {
        MemoryContext   oldcontext;
        int result_count = 0;

        // create a function context for cross-call persistence
        funcctx = SRF_FIRSTCALL_INIT();

        // switch to memory context appropriate for multiple function calls
        oldcontext = MemoryContextSwitchTo( funcctx->multi_call_memory_ctx );

        DBG("iteration: %u", PG_GETARG_INT32(4));

        ret = solve_trash_collection(
                  text2char( PG_GETARG_TEXT_P( 0 ) ), // containers
                  text2char( PG_GETARG_TEXT_P( 1 ) ), // otherlocs
                  text2char( PG_GETARG_TEXT_P( 2 ) ), // vehicles
                  text2char( PG_GETARG_TEXT_P( 3 ) ), // ttimes
                  PG_GETARG_INT32(4),                 // interation
                  &result,
                  &result_count );

        DBG( "solve_trash_collection returned %i", ret );

        if ( ret < 0 ) {
            if ( result ) free( result );

            ereport( ERROR, ( errcode( ERRCODE_E_R_E_CONTAINING_SQL_NOT_PERMITTED ),
                              errmsg( "Unknown Error computing solution!" ) ) );
        }

        #ifdef VRPDEBUG
        DBG( "   Result count: %i", result_count );

        if ( ret >= 0 ) {
            double total_time = 0.0;
            int i;

            for ( i = 0; i < result_count; i++ ) {
                total_time += result[i].deltatime;
		DBG("reult[%i] (seq,vid,nid,ntype,deltatime,cargo)=(%i,%i,%i,%i,%f,%f)",i,result[i].seq,result[i].vid,result[i].nid,result[i].ntype,result[i].deltatime,result[i].cargo);
            }

            DBG( "Total Travel Time: %f", total_time );
        }

        #endif

        // total number of tuples to be returned
        funcctx->max_calls = result_count;
        funcctx->user_fctx = result;

        /* Build a tuple descriptor for our result type */
        if ( get_call_result_type( fcinfo, NULL, &tuple_desc ) != TYPEFUNC_COMPOSITE )
            ereport( ERROR,
                     ( errcode( ERRCODE_FEATURE_NOT_SUPPORTED ),
                       errmsg( "function returning record called in context "
                               "that cannot accept type record" ) ) );

        funcctx->tuple_desc = BlessTupleDesc( tuple_desc );

        MemoryContextSwitchTo( oldcontext );
    }

    // stuff done on every call of the function
    funcctx = SRF_PERCALL_SETUP();

    call_cntr = funcctx->call_cntr;
    max_calls = funcctx->max_calls;
    tuple_desc = funcctx->tuple_desc;
    result = ( vehicle_path_t * ) funcctx->user_fctx;

    // do when there is more left to send
    if ( call_cntr < max_calls ) {
        HeapTuple    tuple;
        Datum        result_data;
        Datum       *values;
        char        *nulls;

        values = palloc( 6 * sizeof( Datum ) );
        nulls = palloc( 6 * sizeof( bool ) );

        values[0] = Int32GetDatum( result[call_cntr].seq );
        nulls[0] = false;
        values[1] = Int32GetDatum( result[call_cntr].vid );
        nulls[1] = false;
        values[2] = Int32GetDatum( result[call_cntr].nid );
        nulls[2] = false;
        values[3] = Int32GetDatum( result[call_cntr].ntype );
        nulls[3] = false;
        values[4] = Float8GetDatum( result[call_cntr].deltatime );
        nulls[4] = false;
        values[5] = Float8GetDatum( result[call_cntr].cargo );
        nulls[5] = false;

        tuple = heap_form_tuple( tuple_desc, values, nulls );

        // make the tuple into a datum
        result_data = HeapTupleGetDatum( tuple );

        // clean up (this is not really necessary)
        pfree( values );
        pfree( nulls );

        SRF_RETURN_NEXT( funcctx, result_data );
    }
    // do when there is no more left
    else {
        DBG( "Going to free path" );

        if ( result ) free( result );

        SRF_RETURN_DONE( funcctx );
    }
}



