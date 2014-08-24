# Pickup and Delivery with Time Windows - VRPDPTW

## Description

This trash collection problem is a multi-depot, single vehicle per depot.
The truck leaves the depot collects trash at assigned stops takes the trash
to a dump and then returns back to its depot. It supports time windows and
each truck can be assigned an appropriate capacity. The pickup sites can
have a capacity and time window if needed.

The solver clusters trash containers to the nearest two depots. An initial
construction algorithm will assign containers to depots based on the nearest
depot until the truck is full. Afterward unassigned containers will get
assigned to the nearest truck that it will fit on. Later a TABU search will be
done to optimize the solution.

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
