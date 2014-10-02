# Trash Collection Problem

## Description

This trash collection problem is a multi-depot, hetrogeneous vehicle problem
with time windows. This could also be used for scheduling School Bus pickup or
drop off problems or similar problems of package delivery or distribution.

The truck leaves a starting location and collects orders at assigned stops
takes them them to a drop off site then returns back to its ending location.
It supports time windows and each truck can be assigned an appropriate
capacity. The pickup sites can have a capacity and time window as needed.

## Input data

The input data is organized into four tables

### Collection Sites (Containers)
    * id - a unique positive integer identifying the container (1)
    * x - the x location in meters (2)
    * y - the y location in meters (2)
    * open - earliest time for collection (3)
    * close - latest time for collection (3)
    * service - time required for service (3)
    * demand - the amount of load that will get added to the truck (4)

### Other Locations (Starting locations, Ending locations and dump locations)
    * id - a unique positive integer identifying a location (1)
    * x - the x location in meters (2)
    * y - the y location in meters (2)

### Vehicle definition
    * vid - a unique positive integer identifying the vehicle
    * start_id - an id from Other Locations where the vehicle will start
    * dump_id - an id from Other locations where the vehicle will dump
    * end_id - and id from Other locations where the vehicle will end
    * capacity - the capacity of the vehicle in container demand units (4)

### Travel cost matrix
    * from_id - unique id from either Collection sites or Other locations
    * to_id - unique id from either Collection sites or Other locations
    * cost - the cost to get from location from_id to location to_id (2,3,5)

### Notes
    (1) Collection site ids and Other location ids MUST NOT overlap. The union
        of these ids MUST be unique.
    (2) x, y coordinate values my be in meters (ie: NOT in lat, lon) as we use
        these values to estimate any missing distances or travel times needed.
    (3) open, close and service times are minutes from 00:00 hrs.
    (4) demand and capacity MUST be in consistent units. It can be pounds,
        kilos, people, volume, count of items, we donot care as long as
        it is consistently represented.
    (5) Cost is currently assumed to be in minutes of travel time. If an entry
        is missing in the cost matrix, we will estimate it by computing the
        Euclidean distance in meters and compute the time in minutes based on
        an average speed of 15 Km/hr.

## Data Validation

When we load a problem we will do some basic valiation of the input data and
throw an error if it fails. The following out lines things that will get
rejected and cause us to reject the input:

### Reasons for reject containers

    * has a negative id
    * demand <= 0
    * has an id that is a duplicate of another container
    * has an id that is a duplicate of Other location

### Definition of Vehicles

A vehicle is defined as indicated above: ``vid start_id dump_id end_id capacity``
It is valid to have all these locations be a dump location. But all these ids
MUST be defined in the Other Locations. If you have a Container at one of the
Other Locations then is MUST have a unique id in the container table and a
separate unique id in the Other Locations table.

## Output Format

For the output of the SQL function the vehicle routing problem is basically
just an ordering problem. So we will report the order and the incremental cost
to get to each node in a table format that can then be joined to other data
in the database. The output structure will look like:

    * seq - this is a serial increasing integer the preserves order
    * vid - vehicle id
    * nid - node id
    * cost - delta cost to get to this node id

For example if we had a vehicle path like: 1 -1 0 2 3 4 5 6 7 8 9 0 0 -1
Where -1 is a separator, the initial 1 is the vehicle id, the value following
the -1 is the starting location, the last 0 is the ending location and the
value before the ending location is the dump. In this example, the starting,
ending and dump are all 0. for cost I am assuming 3 min travel and 1 min
service time for containers and 11 min service time at the dump.

The sql output would looke like:

```
    seq|vid|nid|cost
     1   1   0   0
     2   1   2   4
     3   1   3   8
     4   1   4  12
     5   1   5  16
     6   1   6  20
     7   1   7  24
     8   1   8  28
     9   1   9  32
    10   1   0  36
    11   1   0  50
```


## Links to articles

* TBD

## Trashnode Class

This is an extension of the Tweval base class and adds members for tracking the
the nearest two depots and the nearest dump site. It also has an explicit
``ntype`` member to identify if it is a pickup noe, and dump node or a depot
node.

* class members
* class methods


## Vehicle Class

* overview of class
* class members
* class methods

## TrashProblem Class

* overview of class
* class members
* class methods

## Solver Class

* overview of class
* class members
* class methods

---

[Up](./README.md)
