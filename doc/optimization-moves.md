# Route Optimization Moves

The following moves describe various optimization moves that can be applied
to either single routes or multiple routes. These are fairly standard
moves and can be applied during TABU or VLNS search optimization. These
moves are lower level perturbations of the path, but there may be
problem specific constraints like capacity, time windows or pickup before
delivery that need to be assessed at the problem level as opossed to the
path level.

The Figures 1-9 were taken from the paper "Vehicle Routing Problem with Time Windows, Part I: Route Construction and Local Search Algorithms" by OLLI BRÄYSY and MICHEL GENDREAU.

![Figure 1 - Savings Heuristic](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/savings-heuristic.png "Figure 1 - Savings Heuristic")

![Figure 2 - 2-Opt Exchange](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/2-opt-exchange.png "Figure 2 - 2-Opt Exchange")

![Figure 3 - Or-Opt Exchange](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/or-opt-exchange.png "Figure 3 - Or-Opt Exchange")

![Figure 4 - 2-Opt* Exchange](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/2-opt-star-exchange.png "Figure 4 - 2-Opt* Exchange")

![Figure 5 - Relocate Opt](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/relocate-opt.png "Figure 5 - Relocate Opt")

![Figure 6 - Exchange Opt](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/exchange-opt.png "Figure 6 - Exchange Opt")

![Figure 7 - Cross Exchangei Opt](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/cross-exchange-opt.png "Figure 7 - Cross Exchange Opt")

![Figure 8 - GENI Exchange Opt](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/geni-exchange-opt.png "Figure 8 - GENI Exchange Opt")

![Figure 9 - Cyclic Exchange Opt](https://raw.githubusercontent.com/woodbri/vehicle-routing-problems/develop/doc/images/cyclic-transfer-opt.png "Figure 9 - Cyclic Exchange Opt")


In the following examples, assume the depot is ``0`` and the other numbers
are customer nodes. The *before* and *after* routes will show how the
proposed move impacts the ordering of the nodes in the route.

*NOTE: THE DESCRIPTIONS MAY BE WRONG - THE FIGURES ABOVE ARE CORRECT*



## 1-opt move

Remove a node from the path and insert it elsewhere.

### intra-route

Example: Move node 3 after node 5

 * before:

```
0-1-2-3-4-5-0
```

 * after:

```
0-1-2-4-5-3-0
```

### inter-route

Move node 3 from route a to route b before 6.

It should be noted that this is and the n-opt move the only moves that have
the possibility to reduce the number of routes needed in the solution. When
this move results in the elimination of a route the higher level code should
probably insure that the resulting move is valid and does not violate any
constraints.

 * before:

```
a: 0-1-2-3-4-5-0
b: 0-6-7-8-0
```

 * after:

```
a: 0-1-2-4-5-0
b: 0-3-6-7-8-0
```

## n-opt move

Remove a sequence of 2 or more nodes from the path and insert it elsewhere.

### intra-route

Example: Move node 2-3 after node 5

 * before:

```
0-1-2-3-4-5-0
```

 * after:

```
0-1-4-5-2-3-0
```

### inter-route

Example: Move node 2-3 from route a to route b before 6.

It should be noted that this is and the 1-opt move the only moves that have
the possibility to reduce the number of routes needed in the solution. When
this move results in the elimination of a route the higher level code should
probably insure that the resulting move is valid and does not violate any
constraints.

 * before:

```
a: 0-1-2-3-4-5-0
b: 0-6-7-8-0
```

 * after:

```
a: 0-1-4-5-0
b: 0-2-3-6-7-8-0
```

## 2-opt exchange

Exchange a pair of nodes

### intra-route

Example: Exchange (swap) nodes 3 and 5

 * before

```
0-1-2-3-4-5-0
```

 * after:

```
0-1-2-5-4-3-0
```

### inter-route

Example: Exchange node 3 in route a with node 6 in route b

 * before

```
a: 0-1-2-3-4-5-0
b: 0-6-7-8-0
```

 * after

```
a: 0-1-2-6-4-5-0
b: 0-3-7-8-0
```

## k-opt exchange

Exchange a sequence of 2 or more consecutive nodes with another sequence of
nodes

### intra-route

Example: Exchange nodes 1-2 with nodes 4-5

 * before

```
0-1-2-3-4-5-0
```

 * after

```
0-4-5-3-1-2-0
```

Example: Exchange nodes 2-3-4 with nodes 7-8

 * before

```
0-1-2-3-4-5-6-7-8-9-0
```

 * after

```
0-1-7-8-5-6-2-3-4-9-0
```

### inter-route

Example: Exchange nodes 1-2 in route a with nodes 7-8 in route b

 * before

```
a: 0-1-2-3-4-5-0
b: 0-6-7-8-0
```

 * after

```
a: 0-7-8-3-4-5-0
b: 0-6-1-2-0
```

Example: Exchange nodes 2-3-4 in route a with nodes 7-8 in route b

  * before

```
a: 0-1-2-3-4-5-0
b: 0-6-7-8-9-0
```

 * after

```
a: 0-1-7-8-5-0
b: 0-6-2-3-4-9-0
```

