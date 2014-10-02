vehicle-routing-problems
========================

## C++ Classes for solving various vehicle routing problems

 * baseClasses - Classes shared betwen the various problems
 * trash-collection - solve a trash collection problem. This has multiple depots with a single vehicle at each, they go to various containers and collect the trash and unloads at a dump before returning to its home depot. There is support for multiple dump sites, vehicle capacity and time windows.
 * vrpdptw - A single depot multiple vehicle pick and delivery problem with time windows.
 * more to come ...

# THIS IS A WORK IN PROGRESS

# Dependencies

We current this have dependencies in our development environment for

 * libgd2 - used for generating plots of the routes
 * libcurl - used via curlpp to access OSRM engine
 * curlpp - a C++ wrapper around libcurl to talk to OSRM engine
 * OSRM - we have a local server installed for our coverage area. This is used for comput drive times and routes

