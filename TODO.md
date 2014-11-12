# TODO List

This is just a running to do list based on planning discussions. Items
here are just based on current thinking and may change in the future or get 
dropped off the list.

Please post new items at the top of the list.


## 2014-11-11

1) integration of OSRM

 * we have various different levels of dealing with travel time
   a) Euclidean time (compute distance and convert to time)
   b) TraveTime matrix (uses Euclidean time for missing cells)
   c) Osrm time for full paths (but not used for optimization)
   d) Full osrm integration into Tweval (used with optimization)

 * probably should make this a compile time option

 * For d) style integration it needs to happen in TWC, Twnode,
   and propagate up through Twpath and Vehicle

The obvious issue with using Osrm is the hit to performance. The work I have done indicates that we can reduce the performance hit per request from 500 ms per http request down to around 11 ms using a shared memory connection. While this is impressive, we are already experiencing 2 hour run times on the larger problems so adding any additional time may not be acceptable. So integration should be based on compile time options to configure the code.

### Comment/Question

Currently we cache the travel time from the previous node on each node. When we implemented Osrm ttime we need to compute the route from the start to the current node. We current do this be caching all the previous nodes in the form of a URL string the gets appended to for each node in the path.

The new code need to x,y or lon,lat for all the previous nodes in the path (the equivalent of the url). We can store this as a container of nodes preceding this node or a pointer to the previous node, which would allow us to iterate through the previous and assemble as needed.

2) Other task list items

 * source tree reorg for cmake

 * cmake with options and dependency checking
   - boost packages and versions
   - osrm source and version
   - libcurl and curlpp
   - postgresql version and tools

 * code cleanup
   - see postgresql
   - eliminate unused code

 * postgresql integration and testing
   - need to remove all output or redirect to log file
   - need to convert all assert() into error checks
   - need to define and implement an handling

 * osrm tools
   - extract and prep data
   - start and stop server
   - documentation

 * validate input and return issues

 * documentation
   - installation
   - user commands
   - code


