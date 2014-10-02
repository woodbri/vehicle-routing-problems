trash-collection
================

# VRP - Multi-Depot Trash Collection

This Vehicle Routing Problem deals with multiple trash collection vehicles
each with a home location (Multi-Depot, single vehicle per depot). Each
vehicle leaves its home depot, collects waste at designated pickup locations
and make a final stop at the dump to empty its load, before returning to
its home depot location.

 * vehicles can not exceed their designated capacity
 * vehicles make only a single stop at the dump
 * vehicles must work within time windows assigned
 * minimize the overall distance traveled in the solution

## Notes:

 1. currently the problem is setup to support multiple dump locations
 2. vehicles can use any dump location
 3. plan is to integrate this into pgRouting when it works

# THIS IS A WORK IN PROGRESS

