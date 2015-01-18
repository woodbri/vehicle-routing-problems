# Compile and Installation

The compiliation and installation has 1 or 2 steps in the process depending on what you want to build.

 1. compile the core vrptools libraries and the command line executables
 2. compile the postgresql wrappers and link that to the vrptools library

We built the code to work from a standalone command line utility so we can run it as a simple shell command for debugging purposes. These command line tools read data from text files, not the database and can be easily run inside a debugger, or memory leak checker, or performance profiler. We do not expect most users to every look at these, but some developers might be interested, so we mention them here for completeness.

Step 2 above, takes the library file, our class header files, the osrm client files, and various dependancies and builds a postgresql extension called *vrptools* and installs it with the extensions for the database version requested.

## Makefiles

There are two Makefiles that manage the build process.

### Makefile for the core library classes
``Makefile`` manages build the core vehicle_routing_problem source for Step 1 above. *Read* this file and make changes to the defines as needed. We recommend the settings for production use:

```
OSRMBACKEND_DIR = <location where you compiled osrm-backend>
NO_TRACE = yes
WITH_OSRMCLIENT = yes
WITH_STATS =
WITH_MD5 =
WITH_OSRM =
WITH_PLOT =
DEBUG_MODE =

OSRMCLIENT_HEADER =$(BASECLASSES_DIR)/osrmclient.h
OSRMCLIENT_CFLAGS = -DOSRMCLIENT -I$(OSRMBACKEND_DIR)
OSRMCLIENT_LIBS = -L/usr/local/lib -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lOSRM -lUUID -lGITDESCRIPTION -lz -lrt -lboost_filesystem -lboost_iostreams -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lpthread -lstxxl
```

NOTE: ``OSRMCLIENT_LIBS`` must be set to the same value as used when linking the osrm-datastore executable.

Compile and installation for postgresql can be done using the following commands after you have installed all the required dependencies:

```
git clone https://github.com/woodbri/vehicle-routing-problems.git
cd vehicle-routing-problems
git checkout develop    # note this change to a version tag in the future
cd src/trash-prob/trashWithMovingDumps/integration
./make-compile
```

### Makefile.library to integrate with postgresql

This make file has one required parameter ``PGVER=9.2|9.3`` and is used to define which postgresql version you are building against. It also requires that you have already run ``make -f Makefile.vrptools`` or it will remind you to do so.

NOTE: This makefile looks for ``/usr/lib/postgresql/$(PGVER)/bin/pg_config``, if you have not installed postgresql-server-dev-$(PGVER) in this location you will need to edit the ``Makefile.library``.

## Dependencies

### osrm-backend

This is required if you have WITH_OSRMCLIENT set.

Note we built and tested against a fairly old version of osrm-backend as defined by the git checkout command below. osrm-backend is rapidly changing so we recommend that you use the the command below and NOT a newer or older version without testing and verifing that it works as expected.

```
git clone https://github.com/Project-OSRM/osrm-backend.git
cd orsm-backend
git checkout 8f18ba3 -b orsm-tools
mkdir build
cd build
cmake ..
pwd  # to get an appropriate path for 
make  # remember to copy the link parameters needed for OSRMCLIENT_LIBS above
```

### rapidjson

This is required if you have WITH_OSRMCLIENT set.

Osrm returns data as json documents. We use rapidjson C++ library for parsing the json documents. It was not available as a package on our Ubuntu systems, so we installed it with the following commands. It is a header only library so there is nothing to compile and we just copy the directory tree into ``/usr/local/include/rapidjson``.

```
git clone https://github.com/miloyip/rapidjson.git
cd rapidjson
git checkout master
sudo rsync -a include/rapidjson /usr/local/include/.
```

### google-glog

This is required if you have DEBUG_MODE, WITH_STATS, or TRACE_CFLAGS set in Makefile.

This is an application logging facility. If you use any of the options in the mapfile like STATS, DEBUG, TRACING, etc it will require installing this package so it can log the messages to a file on the server. This is *NOT* recommend for production systems as it can generate a larger number of big files that will need to be managed so you do not fill up your disk.

It can be found here: https://code.google.com/p/google-glog/

```
wget https://code.google.com/p/google-glog/downloads/detail?name=glog-0.3.3.tar.gz
tar xzf glog-0.3.3.tar.gz
cd glog-0.3.3
./configure
make
sudo make install
```

NOTE: the make file assumes that this is installed in /usr/local/. If this is not the case please change ``Makefile``.

### libgd2-xpm, libgd2-xpm-dev

This is required if you have WITH_PLOT set in Makefile. On Ubuntu and Debian where can be installed from the system package mamager.

```
sudo apt-get install libgd2-xpm libgd2-xpm-dev
```



# SQL commands vrp_trashCollection() and vrp_trashCollectionCheck()

With your extension built and installed as above. you can now create a database and add the extensions to it.

```
createdb -U postgres -h localhost mytestdb
psql -U postgres -h localhost mytestdb -c "create extension postgis"
psql -U postgres -h localhost mytestdb -c "create extension vrptools"
```

If you built using the ``WITH_OSRM`` rather than the ``WITH_OSRMCLIENT``, then you will also need to load. This also presumes that you have downloaded and built the osrm-tools postgresql extension which can be found at: https://github.com/woodbri/osrm-tools/ in the postgresql directory.

```
psql -U postgres -h localhost mytestdb -c "create extension osrm"
```

This function solves a trash collection problem with multiple hetrogeneous
vehicles that can make multiple runs to a dump. The input is defined in
four tables ``containers``, ``otherlocs``, ``vehicles``, and ``ttimes``
that are passed to the function as SQL queries.

This example queries identifies the required column names for each table.
More detailed information is below.

```
    SELECT seq, vehicle_id, node_id, node_type, delta_time, cargo
      FROM vrp_trashCollection(
        'select id, x, y, open, close, service, demand from containers',
        'select id, x, y, open, close from otherlocs',
        'select vid, start_id, dump_id, end_id, capacity, dumpservicetime,
                starttime, endtime from vehicles',
        'select from_id, to_id, ttime from ttimes'
        );
```

## containers table

Each of the following columns (with these names) must exist in the query for the containers. You actual column names may be different and you can rename them in the query like `select longitude as x, ...`.

Times are represented as unitless deltas from 0. We did all of our developement testing using minutes from 00:00 hours. For example, if a trash collection shift was 6 hours long starting at midnight and all containers were available for pickup during the whole shift, then they would all have `open = 0` and `close = 360` where `6 hrs * 60 min = 360 min`. If it takes 2 minutes to service the container the ` service = 2`.

 * id (integer) - unique integer ID like the table primary key
 * x (double precision) - WGS84 longitude location of container
 * y (double precision) - WGS84 latitude location of container
 * demand (double precision) - The amount of capacity of the truck that will get used by this pickup
 * open (double precision) - The earliest pickup time for this container
 * close (double precision) - The latest pickup time for this container
 * service (double precision) - The time it takes to perform the pickup
 * street_id (integer) - A unigue street ID that the containter has been placed on or -1 for unknown.

Note that if your location data is in another projection like UTM Zone 21N (SRID:32721), then you will need to use something like `st_x(st_transform(geom, 4326)) as x, st_y(st_transform(geom, 4326)) as y, ` to reproject it to WGS84 long-lat.

The `street-id` field is used if it is set and OSRM is not available. We use this to improve the distance calcuations between containers. The results are significantly more accurate when using OSRM and we recommend using that and simply setting this column to -1, like `select ..., -1::integer as street_id, ...`.

## otherlocs table

Other locations are places like the dump locations, starting locations, and ending location of the truck after making its final trip to the dump for its shift. These location might represent despatching depots of the vehicles, or cleaning, repair and refueling centers. The current model assumes a vehicle has a starting location a dump that it will use when it is full, and an ending location it will go to after its final trip the the dump at the end of its shift.

The id values should also be unique with respect to container id's. The intersection of the set of container id and set of otherlocs id should be the empty set. These two tables must not have the same id values.

 * id (integer) - unique integer ID for these non-container locations
 * x (double precision) - WGS84 longitude of the location
 * y (double precision) - WGS84 latitude of the location
 * demand (double precision) - set to 0, its ignored
 * open (double precision) - The earliest available access to this location
 * close (double precision) - The lastest available access to this location

## vehicles table

Each vehicle is defined by the following.

 * vid (integer) - unique integer ID for the vehicle
 * start_id (integer) - the ID from the otherlocs table where the vehicle till start out on its shift
 * dump_id (integer) - the ID from the otherlocs table for the dump the this vehicle will use
 * end_id (integer) - the ID from the otherlocs table where the vehicle will end its shift
 * dumpservicetime (double precision) - the average time that a vehicle will be at the dump to unload its cargo. This should include wait time in a queue in addition to the time it takes to empty its load and get under way again.
 * capacity (double precision) - The capacity of the truck. When the vehicle gets close to this capacity it will be forced to go to the dump to empty its load.
 * starttime (double precision) - Start time for this truck's shift, ie the time it will leave the starting location
 * endtime (double precision) - The time when it MUST arrive at the ending location.

## ttimes table

Given that most of the locations for an give locality are relatively static from from day to day, we can gain a significant performance benefit by precomputing the travel times between locations, rather than computing them as part of the query in each request. This table holds the precomputed values. If the problem needs travel times between locations that are not in the table it will compute them on fly, but it does not save these into the table so if you need those same missing values on the next query it will recompute them again.

 * from_id (integer) - id of path starting point
 * to_id (integer) - id of path ending point
 * ttime (double precision) - travel time to travel from from_id to to_id

## Input Validation Query

Because these queries can take considerable time to run and there is a lot of complex relationships between the input, we have provided a query that analyzes the proposed query input and reports on data errors in the input. These need to be fixed before the query can be run.

```
    SELECT messages
      FROM vrp_trashCollectionCheck(
        'select id, x, y, open, close, service, demand from containers',
        'select id, x, y, open, close from otherlocs',
        'select vid, start_id, dump_id, end_id, capacity, dumpservicetime,
                starttime, endtime from vehicles',
        'select from_id, to_id, ttime from ttimes'
        );
```

The checking follows this process:

  1. First it looks for errors in containers
  2. if it has errors it will stop and pin point all the errors it might have
  3. if no errors are found then it looks for errors in otherlocations
  4. if it has errors it will stop and pin point all the errors it has
  5. then it will look if the same id is used for containers and otherlocations
  6. then it looks for errors in vehicles
  7. that is because its only one string
  8. and I had problems when I made a test with nodes that all were problematic in evrything that I checked
  9. so, i decided to keep the string as samll as possible
  10. so, if you run the check and get container errors only that doesnt mean the other locs and vehicles are correct

