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
#ifndef COMPATIBLE_H
#define COMPATIBLE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <math.h>
#include <limits>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef OSRMCLIENT
#include "osrmclient.h"
#endif

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif

#include "node.h"
#include "twpath.h"
#include "pg_types_vrp.h"
#include "singleton.h"



/*! \class TWC
 * \brief Class TWC (Time Window Compatibility) provides tools for working with Twpath objects.
 *
 * There are two separate concepts that this class deals with. Compatibility
 * refers to Time Window Compatibility (TWC) and reachability that refers to
 * logical connectedness between nodes.
 *
 * \b Compatibility
 *
 * Time windows on node can be narrow or wide. There is a lot of analysis and
 * complexity and evaulating and manipulating node in a path when you need to
 * consider time windows. When all time windows are infinitely wide then there
 * are no time wondow constraints bacause any node can be place anywhere in
 * the path without creating a time window violation. Conversely, the narrower
 * the time windows the mode constrained the problem becomes.
 *
 * This class provides tools for manipulating and analysis of paths with
 * respect to issues of time window compatibility.
 *
 * Many of the ideas in this class are taken from the paper "A sequential
 * insertion heuristic for the initial solution to a constrained vehicle
 * routing problem" by JW Joubert and SJ Claasen, 2004.
 *
 * \b Reachability
 *
 * Reachability is a logical concept, for example think of a pickup and delivery
 * problem, it is logically in consistent to arrive at a delivery node before
 * you have made the related pickup for that delivery and visa versa. These
 * are determined by looking in a travel time matrix for plus infinity which
 * indicates there is no logical connection from source to destination.
 *
 * This also presumes that you have updated the travel time matrix and these
 * values appropriately if you plan to use the reachability functions.
 *
 */
template <class knode> class TWC {
  private:

    typedef TwBucket<knode> Bucket;
    typedef unsigned long int UID;
    typedef unsigned long int POS;

    #ifdef OSRMCLIENT
    /*! \todo */
    typedef struct { //all are ids when prev==from its a 3 node call
        UID prev;
        UID from;
        UID middle;
        UID last;
    } TTindex;
    struct classcomp {
        bool operator() ( const TTindex &lhs, const TTindex &rhs ) const {
            return lhs.prev < rhs.prev ? true : lhs.from < rhs.from ? true :  lhs.middle <
                   rhs.middle ? true : lhs.last < rhs.last;
        };
    };
    typedef std::map<TTindex, double, classcomp>  TT4;
    typedef typename std::map<TTindex, double>::iterator p_TT4;

    #endif



    TwBucket<knode> original;
    mutable std::vector<std::vector<double> > twcij;
    mutable std::vector<std::vector<double> > travel_Time;

    inline double _MIN() const { return -std::numeric_limits<double>::max();};
    inline double _MAX() const { return std::numeric_limits<double>::max();};

    #ifdef OSRMCLIENT
    mutable TT4 travel_Time4;
    #endif

  public:

    /*! \brief cleans all the tables, leaving them blanck for a next execution */
    void cleanUp() {
	original.clear();
	twcij.clear();
	travel_Time.clear();
        #ifdef OSRMCLIENT
    	travel_Time4.clear();
    	#endif
    }
	
	


    // -------------------  major tools  ----------------------------

    /*!
     * \brief Searches a bucket of unassigned nodes for one that is nearest to the exist path and compatible with time windows.
     *
     * For each node in the unassigned bucket, check the time window
     * compatibility for each existing node in the path can compute the
     * shortest distance to that node. We save the node, position and
     * distance for the node with the shortest distance and return them.
     *
     * The best node is returned if found, but it is not removed from
     * the unassigned bucket and it is not added to the truck.
     *
     * \param[in] truck The truck that we want to find the best node for.
     * \param[in] unassigned The bucket of unassined nodes to look through.
     * \param[out] pos The position in the truck to insert the best node.
     * \param[out] bestNode The best node from the unassigned bucket.
     * \param[out] bestDist The distance to the segment for the best node.
     * \return True if it found a compatible node, False otherwise.
     */
    bool  findNearestNodeTo( const TwBucket<knode> &truck,
                             const  TwBucket<knode> &unassigned,
                             POS &pos,
                             knode &bestNode,
                             double &bestDist ) const {
        assert( unassigned.size() );
        int flag = false;
        bestDist = _MAX();   // dist to minimize
        pos = 0;        // position in path to insert
        double d;


        for ( int i = 0; i < unassigned.size(); i++ ) {
            for ( int j = 0; j < truck.size() - 1; j++ ) {
                if ( isCompatibleIAJ( truck[j], unassigned[i], truck[j + 1] ) ) {
                    d = truck.segmentDistanceToPoint( j , unassigned[i] );

                    if ( d < bestDist ) {
                        bestDist = d;
                        pos = j + 1;
                        bestNode = unassigned[i];
                        flag = true;
                    }
                }
            }
        }

        return flag;
    }

    bool  findNearestNodeUseExistingData( const TwBucket<knode> &truck,
                                          const  TwBucket<knode> &unassigned,
                                          POS &pos,
                                          knode &bestNode,
                                          double &bestDist ) const {
        assert( unassigned.size() );
        int flag = false;
        bestDist = _MAX();   // dist to minimize
        pos = 0;        // position in path to insert
        double d;


        for ( int i = 0; i < unassigned.size(); i++ ) {
            for ( int j = 0; j < truck.size() - 1; j++ ) {
                if ( not ( j ==
                           0 ) // all nodes from the depot that are missing should be calculated
                     and (   ( travel_Time[ truck[j].getnid() ][ unassigned[i].getnid() ] == -1 )
                             or ( travel_Time[ unassigned[i].getnid() ][ truck[j + 1].getnid() ] == -1 ) ) )
                    continue;

                if ( isCompatibleIAJ( truck[j], unassigned[i], truck[j + 1] ) ) {
                    d = truck.segmentDistanceToPoint( j , unassigned[i] );

                    if ( d < bestDist ) {
                        bestDist = d;
                        pos = j + 1;
                        bestNode = unassigned[i];
                        flag = true;
                    }
                }
            }
        }

        if ( not flag ) return findNearestNodeTo( truck, unassigned, pos, bestNode,
                                   bestDist ) ;

        return flag;
    }

    /*!
     * \brief Select all nodes in a bucket from which you can not reach node id \b to.
     *
     * Given a bucket of nodes, return all the nodes from which you can not
     * reach a given node because of logical inconsistancies.
     *
     * For example, if a node is a delivery node then we can not reach the
     * pickup node associated with the delivery from the delivery node. IE:
     * you can never travel from the deliver node to the pickup node of the
     * same order.
     *
     * \param[in] nodes The source nodes from which we want to get to node \b to.
     * \param[in] to The target node id we are trying to get to.
     * \return A bucket of nodes from which you are unable to reach node to.
     */
    TwBucket<knode> getUnreachable( const TwBucket<knode> &nodes, UID to ) const {
        assert( to < original.size() );
        Bucket unreachable;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( not isReachableIJ( nodes[i].getnid(), to ) )
                unreachable.push_back( nodes[i] );

        return unreachable;
    }

    /*!
     * \brief Select all nodes in a bucket that are not reachable from the given node.
     *
     * Given a bucket of nodes, return all the nodes that are not reachable
     * from the input node because of logical constraints.
     *
     * \param[in] from The source node id that we want to leave from to get to nodes in the bucket.
     * \param[in] nodes The bucket of nodes we want to get to.
     * \return A bucket of nodes that are not reachable from \b from
     */
    TwBucket<knode> getUnreachable( UID from, const TwBucket<knode> &nodes ) const {
        assert( from < original.size() );
        Bucket unreachable;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( not isReachableIJ( from, nodes[i].getnid() ) )
                unreachable.push_back( nodes[i] );

        return unreachable;
    }

    /*!
     * \brief Select all nodes in a bucket from which we can reach node id \b to
     *
     * Given a bucket of nodes, return all the nodes from which we can reach
     * node is \b to without creating logical inconsistencies.
     *
     * \param[in] nodes The bucket of nodes we want to test if we can get to node id \b to
     * \param[in] to The target node id we want to get to.
     * \return A bucket of nodes from which we can get to node id \b to
     */
    TwBucket<knode> getReachable( const TwBucket<knode> &nodes, UID to ) const {
        assert( to < original.size() );
        Bucket reachable;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( isReachableIJ( nodes[i].getnid(), to ) )
                reachable.push_back( nodes[i] );

        return reachable;
    }

    /*!
     * \brief Select all the nodes in the bucket that we can get to from node id \b from
     *
     * Given a source node id and a bucket of nodes, return all the nodes in
     * the bucket that are directly reachable from node id \b from.
     *
     * \param[in] from The node id for the source node.
     * \param[in] nodes A bucket of potential target nodes.
     * \return A bucket of nodes that are reachable from node if \b from
     */
    TwBucket<knode> getReachable( UID from, const TwBucket<knode> &nodes ) const {
        assert( from < original.size() );
        Bucket reachable;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( isReachableIJ( from, nodes[i].getnid() ) )
                reachable.push_back( nodes[i] );

        return reachable;
    }

    /*
     * \brief Select nodes that are not compatible as predecessors to node id \b to.
     *
     * Given a bucket of nodes, return all nodes that can not be predecessors to
     * node id \b to because of time window incompatibility.
     *
     * \param[in] nodes A bucket of nodes to test for incompatiblity.
     * \param[in] to The node id that we want to get to.
     * \return A bucket of nodes that are incompatible as predecessors to node id \b to.
     */
    TwBucket<knode> getIncompatible( const TwBucket<knode> &nodes, UID to ) const {
        assert( to < original.size() );
        Bucket incompatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( not isCompatibleIJ( nodes[i].getnid(), to ) )
                incompatible.push_back( nodes[i] );

        return incompatible;
    }

    /*!
     * \brief Select all nodes that are not compatible as successors to node id \b from.
     *
     * Given a bucket of nodes, return all that can not be successors to node
     * id \b from because of time window incompatibilities.
     *
     * \param[in] from The source node id.
     * \param[in] nodes A bucket of potential successor nodes.
     * \return A bucket of incompatible successor nodes.
     */
    TwBucket<knode> getIncompatible( UID from,
                                     const TwBucket<knode> &nodes ) const {
        assert( from < original.size() );
        Bucket incompatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( not isCompatibleIJ( from, nodes[i].getnid() ) )
                incompatible.push_back( nodes[i] );

        return incompatible;
    }

    /*!
     * \brief Select all nodes that are compatible as predecessor nodes to node id \b to.
     *
     * Given a bucket of nodes, return all can be predecessor nodes to node
     id \b to besaed on time window compatibility.
     *
     * \param[in] nodes A bucket of potential predecessor nodes.
     * \param[in] to The node id that we want to be a successor of nodes in the bucket.
     * \return A bucket of nodes that can be predecessor nodes to node id \b to.
     */
    TwBucket<knode> getCompatible( const TwBucket<knode> &nodes, UID to ) const {
        assert( to < original.size() );
        Bucket compatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( isCompatibleIJ( nodes[i].getnid(), to ) )
                compatible.push_back( nodes[i] );

        return compatible;
    }

    /*!
     * \brief Select all nodes in a bucket that are compatible as successor nodes to node id \b from.
     *
     * Given a bucket of nodes, return all nodes that are compatible as a
     * successor to node id \b from.
     *
     * \param[in] from The node id that that we are looking for successor to.
     * \param[in] nodes A bucket of potential successor nodes.
     * \return A bucket of compatible successors to node id \b from.
     */
    TwBucket<knode> getCompatible( UID from, const TwBucket<knode> &nodes ) const {
        assert( from < original.size() );
        Bucket compatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( isCompatibleIJ( from, nodes[i].getnid() ) )
                compatible.push_back( nodes[i] );

        return compatible;
    }


    /*! \todo comments   */
  private:
    double getTravelTime( UID from, UID to ) const { //this one does all the work gets &sets if needed

        assert( from < original.size() and to < original.size() );
        double time;
        #ifndef OSRMCLIENT
        if ( travel_Time[from][to] == -1 ) {
                time = original[from].distance( original[to] ) / 250;

		#ifdef VRPMINTRACE
                	DLOG(INFO) << "->>>("<<from<<","<<to<<")="<<time<<"\n";
			original[from].dump();
			original[to].dump();
		#endif

                if ( not sameStreet( from, to ) ) {
                    time = time *
                           ( std::abs( std::sin( gradient( from, to ) ) ) +
                             std::abs( std::cos( gradient( from, to ) ) )
                           );
                }
		#ifdef VRPMINTRACE
               	DLOG(INFO) << "----->>>("<<from<<","<<to<<")="<<time<<"\n";
		#endif
        	travel_Time[from][to] = time;
        }
	return travel_Time[from][to];
	#else


        bool oldStateOsrm = osrm->getUse();

        if ( travel_Time[from][to] == -1 ) {
            #ifdef DOSTATS
            STATS->inc( "TWC::getTravelTime(2 parameters) travel_time==-1" );
            #endif

            osrm->useOsrm( true );
            if ( not osrm->getOsrmTime( original[from], original[to], time ) ) {
                time = original[from].distance( original[to] ) / 250;

                if ( not sameStreet( from, to ) ) {
                    time = time *
                           ( std::abs( std::sin( gradient( from, to ) ) ) +
                             std::abs( std::cos( gradient( from, to ) ) )
                           );
                }

                #ifdef DOSTATS
                STATS->inc( "TWC::getTravelTime(2 parameters) euclidean calculated" );
                #endif
            }
            else {
                #ifdef DOSTATS
                STATS->inc( "TWC::getTravelTime(2 parameters) osrm calculated" );
                #endif
            }

            osrm->useOsrm( oldStateOsrm );
            travel_Time[from][to] = time;
            getTwcij( from, to, time );

            //TODO if is not compatible set travel_time as infinity
        }

        return travel_Time[from][to];
        #endif
    }

  public:
    /*!
     * \brief Fetch the travel time from node id \b from to node id \b to. (interface)
     *
     * The travel time matrix is and N x N containter that holds the travel
     * times. This caching in a matrix provides a performance boost, but
     * it also consumes a lot of memory (node_count * node_count *
     * sizeof(double)). The user should check if the returned value is negative
     * which indicates that there is not path between the nodes.
     *
     * \warning Presumes that the travel time matrix has been loaded.
     *
     * \param[in] from Node id of the from node.
     * \param[in] to Node id of the to node.
     * \return The value from the travel time matrix.
     */
    double TravelTime( UID from, UID to ) const {
        return getTravelTime( from, to );
    }

    /*!
     * \brief Fetch the travel time from node \b from to node \b to (interface).
     *
     * Fetch results from a preloaded travel time matrix.
     *
     * \warning Presumes that the travel time matrix has been loaded.
     *
     * \param[in] from Node of the from node.
     * \param[in] to Node of the to node.
     * \return The travel time or plus infinity if the node is not reachable.
     */
    double TravelTime( const knode &from, const knode &to ) const {
        return getTravelTime( from.getnid(), to.getnid() );
    }


    /*! \todo comments   */
  private:
    //this one does all the work
    double getTravelTime( UID prev, UID from, UID middle, UID to ) const {
        assert( prev < original.size() and from < original.size()
                and middle < original.size() and to < original.size() );
        #ifndef OSRMCLIENT

        if ( prev == from ) return  getTravelTime( from,
                                        middle ) + getTravelTime( middle, to );
        else return getTravelTime( prev, from ) + getTravelTime( from,
                        middle ) + getTravelTime( middle, to );

        #else

        if ( prev == from and from == middle ) return TravelTime( middle, to );

        TTindex index = {prev, from, middle, to};
        p_TT4 it = travel_Time4.find( index );

        if ( it != travel_Time4.end() ) {
            #ifdef DOSTATS

            if ( prev == from )
                STATS->inc( "TWC::getTravelTime(3 parameters) 4 dim Table access" );
            else  STATS->inc( "TWC::getTravelTime(4 parameters) 4 dim Table access" );

            #endif
            return it->second;
        }

        double time;

        if ( prev == from ) {
            if ( TravelTime( from, middle ) == TravelTime( middle, from )
                 or TravelTime( middle, to ) == TravelTime( to, middle )
                 or TravelTime( from, to ) == TravelTime( to, from ) ) {
                if ( osrm->getOsrmTime( original[from], original[middle], original[to],
                                        time ) ) {
                    #ifdef DOSTATS
                    STATS->inc( "TWC::getTravelTime(3 parameters) osrm Calculated" );
                    #endif
                    travel_Time4[index] = time;
                    return time;
                }
                else {

                    #ifdef DOSTATS
                    STATS->inc( "TWC::getTravelTime(3 parameters) 2 dim Table access" );
                    #endif
                    return getTravelTime( from, middle ) + getTravelTime( middle, to );
                }
            }
            else {
                #ifdef DOSTATS
                STATS->inc( "TWC::getTravelTime(3 parameters) combination based on 1 way streets" );
                #endif
                //assuming that all 3 nodes are in single way street all times are the best (to avoid calls to ORSM)
                //no need to make all combinations
                time =  getTravelTime( from, middle ) + getTravelTime( middle, to );
                travel_Time4[index] =  time;
                return time;
            }
        }

        if ( TravelTime( prev, from ) == TravelTime( from, prev )
             or TravelTime( from, middle ) == TravelTime( middle, from )
             or TravelTime( middle, to ) == TravelTime( to, middle )
             or TravelTime( from, to ) == TravelTime( to, from ) ) {
            if ( osrm->getOsrmTime( original[prev], original[from], original[middle],
                                    original[to], time ) ) {
                #ifdef DOSTATS
                STATS->inc( "TWC::getTravelTime(4 parameters) osrm Calculated" );
                #endif
                travel_Time4[index] = time;
                return time;
            }
            else  {
                time =  getTravelTime( prev, from ) + getTravelTime( from,
                        middle ) + getTravelTime( middle, to );
                #ifdef DOSTATS
                STATS->inc( "TWC::getTravelTime(4 parameters) 2 dim Table access" );
                #endif
                return time;
            }
        }
        else {
            #ifdef DOSTATS
            STATS->inc( "TWC::getTravelTime(4 parameters) combination based on 1 way streets" );
            #endif
            //assuming that all 3 nodes are in single way street all times are the best (to avoid calls to ORSM)
            //no need to make all combinations
            time =  getTravelTime( prev, prev, from, middle ) + getTravelTime( middle, to );
            travel_Time4[index] =  time;
            return time;
        }

        #endif
    }


  public:
    //this one is an interface , the previous one is the one that does all the work
    double TravelTime( UID from, UID middle, UID to ) const {
        return getTravelTime( from, from, middle, to );
    }

    //this one is an interface, the other one is the one that does all the work
    double TravelTime( const knode &from, const knode &middle,
                       const knode &to ) const {
        return getTravelTime( from.getnid(), from.getnid(), middle.getnid(),
                              to.getnid() );
    }

    //this one is an interface, the other one is the one that does all the work
    double TravelTime( const knode &prev, const knode &from, const knode &middle,
                       const knode &to ) const {
        return getTravelTime( prev.getnid(), from.getnid(), middle.getnid(),
                              to.getnid() );
    }

    //this one is an interface, the other one is the one that does all the work
    double TravelTime( UID prev, UID from, UID middle, UID to ) const {
        return getTravelTime( prev, from, middle, to );
    }


















    /*!
     * \brief Fetch the time window compatibility value traveling from fromNid to toNid.
     *
     * Return the compatibility of servicing customer toNid directly after
     * fromNod. Higher values represent better compatibility of the two time
     * windows being considered. And incompatible time windows will have
     * negative infinity.
     *
     * \param[in] fromNid Nid of the from node.
     * \param[in] toNid Nid of the to node.
     * \return The time window compatibility of traveling directly \b fromNid to \b toNide.
     */
    double compatibleIJ( UID fromNid, UID toNid ) const {
        assert( fromNid < original.size() and toNid < original.size() );
        return  getTwcij( fromNid, toNid ) ;
    }

    /*!
     * \brief Fetch a node by its node id from the original container of nodes.
     *
     * \param[in] nid The node id of the desired node.
     * \return A copy of the node from the original container of nodes.
     */
    const knode &node( UID nid ) const {
        assert( nid < original.size() );
        return original[nid];
    };

    /*!
     * \brief Fetch a node by its node id from the original container of nodes.
     *
     * \param[in] nid The node id of the desired node.
     * \return A copy of the node from the original container of nodes.
     */
    const knode &getNode( UID nid ) const {
        assert( nid < original.size() );
        return original[nid];
    };

    // ---------------- state -----------------------------------

    /*!
     * \brief Report if traveling fromNid to toNide is compatibly.
     *
     * \param[in] fromNid Nid of the from node.
     * \param[in] toNid Nid of the to node.
     * \return True if compatibile, false otherwise.
     */
    bool isCompatibleIJ( UID fromNid, UID toNid ) const {
        assert( fromNid < original.size() and toNid < original.size() );
        return not ( getTwcij( fromNid, toNid )  == _MIN() );
    }

    /*!
     * \brief Report toNide is logically reachable from fromNid.
     *
     * \param[in] fromNid Nid of the from node.
     * \param[in] toNid Nid of the to node.
     * \return True if reachable, false otherwise.
     */
    bool isReachableIJ( UID fromNid, UID toNid ) const {
        assert( fromNid < original.size() and toNid < original.size() );
        return not ( TravelTime( fromNid, toNid )  == _MAX() );
    }


    /*!
     * \brief Report compatibility of traveling fromNid to middleNid to toNid.
     *
     * Check time window compatibility for traveling through a three node
     * sequence of fromNid to middleNid to toNid.
     *
     * \param[in] fromNid First node id in three node sequence to be checked.
     * \param[in] middleNid Second node id in three node sequence to be checked.
     * \param[in] toNid Third node id in three node sequence to be checked.
     * \return True if it is compatible to travel fromNid to middleNid to toNid.
     * \bug I (vicky)  dont think transitivity applies, and I think the process is more complex
     */
    bool isCompatibleIAJ( UID fromNid, UID middleNid, UID toNid ) const {
        assert( fromNid < original.size() and middleNid < original.size()
                and toNid < original.size() );
        isCompatibleIJ( fromNid, middleNid );
        isCompatibleIJ( middleNid, toNid );
        return isCompatibleIJ( fromNid, middleNid )
               and isCompatibleIJ( middleNid, toNid );
    }


    /*!
     * \brief Report compatibility of traveling through a three node sequence.
     *
     * Check time window compatibility for traveling through a three node
     * sequence of \b from node to \b middle node to \b to node.
     *
     * \param[in] from First node in three node sequence to be checked.
     * \param[in] middle Second node in three node sequence to be checked.
     * \param[in] to Third node in three node sequence to be checked.
     * \return True if it is compatible to travel fromNid to middleNid to toNid.
     */
    bool isCompatibleIAJ( const knode &from, const knode &middle,
                          const knode &to ) const {
        return isCompatibleIAJ( from.getnid(), middle.getnid() , to.getnid() );
    }

    // ----------------- The best or the worses -----------------------

    /*!
     * \brief Search a bucket of nodes for the one with the best travel time.
     *
     * Given a bucket of nodes and a from node id, search the bucket for the
     * node that gives the best travel time and is Reachable from the node id.
     * If all nodes are unReachable then just return the first node in the
     * bucket.
     *
     * \param[in] from Node id from which we want the best travel time to a node in \b nodes.
     * \param[in] nodes A bucket of nodes that we want to search.
     * \return The node with the best travel time from node id \b from.
     * \return Or the first node in the bucket if all are unReachable.
     */
    knode findBestTravelTime( UID from, const Bucket &nodes ) const {
        assert ( nodes.size() and from < original.size() );
        Bucket reachable = getReachable( from, nodes );

        if ( not reachable.size() ) return nodes[0];

        knode best = reachable[0];
        double bestTime = _MAX();

        for ( int i = 0; i < reachable.size(); i++ ) {
            if ( reachable[i].getnid() != from
                 and travelTime( from, reachable[i].getid() ) < bestTime ) {
                best = reachable[i];
                bestTime = travelTime( from, reachable[i].getid() );
            }
        }

        return best;
    }

    /*!
     * \brief Search a bucket for the node with the best travel time to node id \b to.
     *
     * Given a bucket of nodes and a to node id, search the bucket for the
     * node that gives the best travel time and is Reachable to arrive at node
     * id \b to.
     *
     * \param[in] nodes A bucket of nodes that we want to search.
     * \param[in] to Node id to which we want the best travel time from a node in \b nodes.
     * \return The node with the best travel time to node id \b to.
     * \return Or the first node in the bucket if all are unReachable.
     */
    knode findBestTravelTime( const Bucket &nodes, UID to ) const {
        assert ( nodes.size() and to < original.size() );
        Bucket reachable = getReachable( nodes, to );

        if ( not reachable.size() ) return nodes[0];

        knode best = reachable[0];
        double bestTime = _MAX();

        for ( int i = 0; i < reachable.size(); i++ ) {
            if ( reachable[i].getnid() != to
                 and travelTime( reachable[i].getid(), to ) < bestTime ) {
                best = reachable[i];
                bestTime = travelTime( reachable[i].getid(), to );
            }
        }

        return best;
    }

    /*!
     * \brief Search a bucket for the node with the worst travel time from node id \b from.
     *
     * Given a bucket of nodes and a from node id, search the bucket for the
     * node that gives the worst travel time and is Reachable departing from
     * node id \b from.
     *
     * \param[in] from The from node id for the search.
     * \param[in] nodes A bucket of nodes to search through.
     * \return The node with the best travel time from node id \b from.
     * \return Or the first node in the bucket if all are unReachable.
     */
    knode findWorseTravelTime( UID from, const Bucket &nodes ) const {
        // from the reachable nodes finds the worse
        assert ( nodes.size() and from < original.size() );
        Bucket reachable = getReachable( from, nodes );

        if ( not reachable.size() ) return nodes[0];

        knode worse = reachable[0];
        double worseTime = _MIN();

        for ( int i = 0; i < reachable.size(); i++ ) {
            if ( reachable[i].getnid() != from
                 and travelTime( from, reachable[i].getid() ) > worseTime ) {
                worse = reachable[i];
                worseTime = travelTime( from, reachable[i].getid() );
            }
        }

        return worse;
    }

    /*!
     * \brief Search a bucket for the node with the worst travel time to node id \b to.
     *
     * Given a bucket of nodes and a from node id, search the bucket for the
     * node that gives the worst travel time and is Reachable arriving at
     * node id \b to.
     *
     * \param[in] nodes A bucket of nodes to search through.
     * \param[in] to The from node id for the search.
     * \return The node with the best travel time to node id \b to.
     * \return Or the first node in the bucket if all are unReachable.
     */
    knode findWorseTravelTime( const Bucket &nodes, UID to ) const {
        // from the reachable nodes finds the worse
        assert ( nodes.size() and to < original.size() );
        Bucket reachable = getReachable( nodes, to );

        if ( not reachable.size() ) return nodes[0];

        knode worse = reachable[0];
        double worseTime = _MIN();

        for ( int i = 0; i < reachable.size(); i++ ) {
            if ( reachable[i].getnid() != to
                 and travelTime( reachable[i].getid(), to ) > worseTime ) {
                worse = reachable[i];
                worseTime = travelTime( reachable[i].getid(), to );
            }
        }

        return worse;
    }

    /*!
     * \brief Get a seed node id from a bucket of nodes for creating an new route.
     *
     * The goal is to pick a node from the bucket based on the time window
     * compatibility of that node with respect to the other nodes in the bucket.
     *
     * \warning ec2 get seed (needs revision)
     * \warning This code returns a node id, not clear based how it works!
     *
     * \param[in] foo Unused
     * \param[in] nodes A bucket of nodes from which we want to select a seed.
     * \return The node id of a seed node.
     */
    int getSeed( int foo, const Bucket &nodes ) const {
        //ec2 get seed (needs revision)
        int bestId, count;
        double bestEc2;
        int Id;
        int bestCount = 0;
        bestEc2 = - _MIN();

        for ( int i = 0; i < nodes.size(); i++ ) {
            if ( i == 0 ) bestId = nodes[0].getnid();

            Id = nodes[i].getnid();
            count = 0;

            for ( int j = 0; j < nodes.size(); j++ ) {
                if ( i != j and  isCompatibleIJ( Id , nodes[j].getnid() ) ) count++;

                bestCount = count;
                bestId = Id;
            }
        }

        return bestId;
    }

    /*!
     * \brief Get the most TW compatable node as a successor from the bucket.
     *
     * Given a bucket of nodes and from node id, search the bucket for the
     * node that is most TW compatible as a successor to node id.
     *
     * \param[in] fromNid The reference node id.
     * \param[in] nodes A bucket of nodes to search through.
     * \return The node id of the most most compatible node, or
     * \return -1 if empty bucket or there are no compatible nodes.
     */
    int  getBestCompatible( UID fromNid, const Bucket &nodes ) const {
        assert( fromNid < original.size() );
        UID bestId;
        UID toId;

        if ( nodes.empty() ) return -1;

        bestId = nodes[0].getnid();

        for ( int j = 0; j < nodes.size(); j++ ) {
            toId = nodes[j].getnid();

            if ( getTwcij( fromNid, toId ) > getTwcij( fromNid, bestId ) ) {
                bestId = toId;
            }
        }

        if ( compat( fromNid, bestId ) != _MIN() ) return bestId;
        else return -1;
    }

    /*!
     * \brief Compute the total TW compatibility for node id \c at in a bucket.
     *
     * Compute the total TW compatibility for node id \b at. The node with
     * the lowest total TW compatibility should be select as the seed for
     * a new route during initial construction.
     *
     * \sa Equation 2. in reference article
     *
     * \warning Review code against reference article, because the Equation 2
     *          does not match what this code is doing.
     *
     * \param[in] at Node id to compute the total compatibility for.
     * \param[in] nodes A bucket of nodes containing node id \b at.
     * \return The total TW compatibility value for node id \b at.
     */
    double ec2( UID at, const Bucket &nodes ) {
        assert( at < original.size() );
        double ec2_tot = 0;

        for ( int j = 0; j < nodes.size(); j++ ) {
            if ( not ( getTwcij( at, j )  == _MIN() ) ) ec2_tot += getTwcij( at, j );

            if ( not ( getTwcij( j, at )  == _MIN() ) ) ec2_tot += getTwcij( j, at );
        };

        if ( getTwcij( at, at ) == _MIN() ) ec2_tot -= getTwcij( at, at );

        return ec2_tot;
    }


    // ---------------- counting ---------------------------------


    /*!
     * \brief Count the number of nodes that are incompatible from a node.
     *
     * \param[in] at A reference node id to be used in counting.
     * \param[in] nodes A bucket that is to be used in counting.
     * \return The number of nodes that are incompatible from node id \b at.
     */
    int countIncompatibleFrom( UID at, const Bucket &nodes ) {
        assert( at < original.size() );
        int count = 0;

        for ( UID j = 0; j < nodes.size(); j++ ) {
            if ( getTwcij( at, j )  == _MIN() ) count++;
        }

        return count;
    }

    /*!
     * \brief Count the number of nodes that are incompatible as sucessors of a node.
     *
     * \param[in] at A reference node id to be used in counting.
     * \param[in] nodes A bucket that is to be used in counting.
     * \return The number of nodes that are incompatibleas sucessors to node id \b at.
     */
    int countIncompatibleTo( UID at, const Bucket &nodes ) {
        int count = 0;

        for ( UID j = 0; j < nodes.size(); j++ ) {
            if ( getTwcij( j, at )  == _MIN() ) count++;
        }

        return count;
    }



    // ------------------------ DUMPS --------------------------

    #ifdef DOVRPLOG

    /*! \brief Print the original nodes.  */
    void dump() const  {
        assert( original.size() );
        dump( original );
    }


    /*!
     * \brief Print the nodes in the bucket.
     * \param[in] nodes A bucket of nodes to print.
     */
    void dump( const Bucket &nodes ) const  {
        assert( original.size() );
        dumpCompatability( original );
        dumpTravelTime( original );
    }

    /*! \brief Print the TW Compatibility matrix for the original nodes.  */
    void dumpCompatability() const  {
        assert( original.size() );
        dumpCompatability( original );
    }

    /*!
     * \brief Print the TW Compatibility matrix for the input bucket.
     * \param[in] nodes  A bucket of nodes to print the TWC matrix.
     */
    void dumpCompatability( const Bucket &nodes ) const  {
        std::stringstream ss;
        assert( nodes.size() );

        ss.precision( 8 );
        ss << "COMPATABILITY TABLE \n\t";

        for ( int i = 0; i < nodes.size(); i++ )
            ss << "nid " << nodes[i].getnid() << "\t";

        ss << "\n\t";

        for ( int i = 0; i < nodes.size(); i++ )
            ss << "id " << nodes[i].getid() << "\t";

        ss << "\n";

        for ( int i = 0; i < nodes.size(); i++ ) {
            ss << nodes[i].getnid() << "=" << nodes[i].getid() << "\t";

            for ( int j = 0; j < nodes.size(); j++ ) {
                if ( twcij[i][j] !=  -std::numeric_limits<double>::max() )
                    ss << twcij[i][j] << "\t";
                else ss << "--\t";
            }

            ss << "\n";
        }

        DLOG( INFO ) << ss.str();
    }

    /*!
     * \brief Print the travel time matrix for the original nodes.
     */
    void dumpTravelTime() const {
        assert( original.size() );
        dumpTravelTime( original );
    }

    /*!
     * \brief
     *
     * \param[in] nodes A bucket of nodes to print the travel time matrix for.
     */
    void dumpTravelTime( const Bucket &nodes ) const {
        std::stringstream ss;
        assert( nodes.size() );
        ss << "\n\n\n\nTRAVEL TIME TABLE \n\t";

        ss.precision( 2 );

        for ( int i = 0; i < nodes.size(); i++ )
            ss << "nid " << nodes[i].getnid() << "\t";

        ss << "\n\t";

        for ( int i = 0; i < nodes.size(); i++ )
            ss << "id " << nodes[i].getid() << "\t";

        ss << "\n";

        for ( int i = 0; i < nodes.size(); i++ ) {
            ss << nodes[i].getnid() << "=" << nodes[i].getid() << "\t";

            for ( int j = 0; j < nodes.size(); j++ ) {
                if ( travel_Time[i][j] !=  _MAX() )
                    ss << travel_Time[i][j] << "\t";
                else
                    ss << "--\t";
            }

            ss << "\n";
        }

        DLOG( INFO ) << ss.str();
    }


    /*!
     * \brief Print the compatibility matrix using an alternate format for the original nodes.
     */
    void dumpCompatible3() const  {
        assert( original.size() );
        dumpCompatible3( original );
    }

    /*!
     * \brief Print the compatibility matrix using an alternate format for the inptu bucket.
     *
     * \param[in] nodes The bucket of nodes to print the TWC for.
     */
    void dumpCompatible3( const Bucket &nodes ) const {
        std::stringstream ss;
        assert( nodes.size() );

        for ( int i = 0; i < nodes.size(); i++ ) {
            for ( int j = 0; j < nodes.size(); j++ ) {
                for ( int k = 0; k < nodes.size(); k++ ) {
                    ss << "\t ( " << nodes[i].getnid() << " , "
                       << nodes[j].getnid() << " , "
                       << nodes[k].getnid() << ") = "
                       << ( isCompatibleIAJ( i, j, k ) ? "COMP" : "not" );
                }

                ss << "\n";
            }
        }

        DLOG( INFO ) << ss.str();
    }
    #endif  //logs

    // ------------ go back to CALCULATED state -------------------

    /*!
     * \brief Recompute the TWC matrix entries for a given node.
     *
     * \param[in] nid The node id to update the TWC matrix entries for.
     */
    void recreateCompatible( UID nid ) {
        assert( nid < original.size() );

        for ( int j = 0; j < twcij.size(); j++ ) {
            twcij[nid][j] = twc_for_ij( original[nid], original[j] );
            twcij[j][nid] = twc_for_ij( original[j], original[nid] );
        }
    }


    // ---- Functions to adjust compatability depending on problem ----


    /*!
     * \brief Recompute the travel time matrix for a given node.
     *
     * \todo This needs to be coded
     *
     * \param[in] nid The node we want to recomute traveltime for.
     * \param[in] mode Flag to indicate mode of calcualtion
     *                 - 0 = Euclidean point to point
     *                 - 1 = Haversine distance for lat-lon values
     *                 - 2 = Call getTimeOSRM()
     */
    void recreateTravelTime( UID nid, int mode ) {
        assert( "needs to be re-read from file" == "" );
    }


    /*!
     * \brief Set TWC from fromNid to toNid to be incompatible & unreachable.
     *
     * \param[in] fromNid The predecessor node id.
     * \param[in] toNid The successor node id.
     */
    void setIncompatible( UID fromNid, UID toNid ) {
        assert( fromNid < original.size() and toNid < original.size() );
        twcij[fromNid][toNid] = _MIN();
        travel_Time[fromNid][toNid] =  _MAX();
    }


    /*!
     * \brief Set TWC incompatible  & unreachable from nid to all nodes in the bucket.
     *
     * \param[in] nid The from node id that we want set as incompatible.
     * \param[in] nodes A bucket of successor nodes that are incompatible from \b nid.
     */
    void setIncompatible( UID nid, const Bucket &nodes ) {
        assert( nid < original.size() );

        for ( int j = 0; j < nodes.size(); j++ ) {
            twcij[nid][nodes[j].getnid()] =  _MIN();
            travel_Time[nid][nodes[j].getnid()] =  _MAX();
        }

    }


    /*!
     * \brief Set TWC incompatible to nid for all nodes in the bucket.
     *
     * \param[in] nodes A bucket of predecessor nodes that are incompatible with \b nid.
     * \param[in] nid The successor node id that we want set as incompatible.
     */
    void setIncompatible( const Bucket &nodes, UID &nid ) {
        assert( nid < original.size() );

        for ( int i = 0; i < nodes.size(); i++ ) {
            twcij[nodes[i].getnid()][nid] =  _MIN();
            travel_Time[nodes[i].getnid()][nid] =  _MAX();
        }
    }


    /*!
     * \brief Set the travel time between \b fromNid and \b toNid as unReachable
     *
     * \param[in] fromNid The from node id to set.
     * \param[in] toNid The to node id to set.
     */
    void setUnreachable( UID fromNid, UID toNid ) {
        assert( fromNid < original.size() and toNid < original.size() );
        travel_Time[fromNid][toNid] = _MAX();
    }


    /*!
     * \brief Set all nodes in the bucket as unReachable from nid
     *
     * \param[in] nid The from node id we are set as unReachable.
     * \param[in] nodes A bucket of successor nodes to set as unReachable.
     */
    void setUnreachable( UID nid, const Bucket &nodes ) {
        assert( nid < original.size() );

        for ( int j = 0; j < nodes.size(); j++ )
            travel_Time[nid][nodes[j].getnid()] =  _MAX();
    }

    /*!
     * \brief Set all nodes in the bucket as unReachable predecessors of nid.
     *
     * \param[in] nodes A bucket of predecessor nodes to set as unReachable.
     * \param[in] nid The successor node that is unRechable from the bucket nodes.
     */
    void setUnreachable( const Bucket &nodes, UID &nid ) {
        assert( nid < original.size() );

        for ( int i = 0; i < nodes.size(); i++ )
            travel_Time[nodes[i].getnid()][nid] =  _MAX();
    }


    /*!
     * \brief Assign the bucket of nodes as the TWC original and compute TWC.
     *
     * \param[in] _original A bucket of nodes to assign to TWC class.
     */
    void setNodes( Bucket _original ) {
        original.clear();
        original = _original;
        twcij_calculate();
        assert ( original == _original );
        assert ( check_integrity() );
    }



    /*!
     * \brief Compute the average travel time to a given node.
     *
     * \param[in] from A bucket of nodes to use as the start node.
     * \param[in] to A node to be used as the destination node.
     * \return The average travel time from start to destination.
     */
    double getAverageTime( const Bucket &from, const knode &to ) const {
        assert( to.getnid() < original.size() );
        double time = 0;
        int j = to.getnid();

        for ( int i = 0; i < from.size(); i++ ) {
            time += TravelTime( from[i].getnid() , j ) ;
        }

        time = time / from.size();
        return time;
    }

    /*!
     * \brief Compute the average travel time from a given node.
     *
     * \param[in] from The start node.
     * \param[in] to A bucket of destination nodes.
     * \return The average travel time from start to destination.
     */
    double getAverageTime( const knode &from, const Bucket &to ) const {
        assert( from.getnid() < original.size() );
        double time = 0;
        int j = from.getnid();

        for ( int i = 0; i < to.size(); i++ )
            time += travel_Time[j][ to[i].getnid() ];

        time = time / to.size();
        return time;
    }

    /*!
     * \brief Set tCC for a given node related to a given bucket
     *
     * \todo VICKY Please explain!
     *
     * \param[in] C
     * \param[in] picks
     */
    void settCC ( const knode &C, const Bucket &picks ) {
        int pos = C.getnid();
        travel_Time[pos][pos] = getAverageTime( C, picks );
    }

    /*!
     * \brief Test if two nodes are on the same street.
     *
     * \warning This is dependent on street ids being set.
     *
     * \param[in] i Node id 1
     * \param[in] j Node id 2
     * \return True if both nodes are on the same street.
     */
    bool sameStreet( UID i, UID j ) const {
        assert( i < original.size() and j < original.size() );
        return original[i].sameStreet( original[j] );
    }

    /*!
     * \brief Compute the gradient or slope of line from node i to j
     *
     * \bug This function calls Node::gradient that might divide by zero.
     *
     * \param[in] i Node id 1
     * \param[in] j Node id 2
     * \return The gradient of the line.
     */
    double gradient( UID i, UID j ) const {
        assert( i < original.size() and j < original.size() );
        return original[i].gradient( original[j] );
    }



    void setHints( Bucket &nodes ) {
        #ifdef OSRMCLIENT
        #ifdef DOSTATS
        Timer timer;
        #endif

        for ( int i = 0; i < nodes.size(); i++ ) {
            nodes[i].setHint( original[ nodes[i].getnid() ].getHint() );
        }

        #ifdef DOSTATS
        STATS->addto( "TWC::setHints Cumultaive time:", timer.duration() );
        #endif

        #endif
    }


  private:
    void prepareTravelTime() {
        int siz = original.size();
        travel_Time.resize( siz );

        for ( int i = 0; i < siz; i++ )
            travel_Time[i].resize( siz );

        //travel_Time default value is 250m/min
        for ( int i = 0; i < siz; i++ )
            for ( int j = i; j < siz; j++ ) {
                if ( i == j ) travel_Time[i][i] = 0.0;
                else {
                    travel_Time[i][j] = travel_Time[j][i] = -1.0;
                    #ifndef OSRMCLIENT
                    	travel_Time[i][j] = travel_Time[j][i] = getTravelTime( i, j );
		    	#ifdef VRPMINTRACE
                    		DLOG(INFO) << "("<<i<<","<<j<<")="<<travel_Time[i][j]<<"\n";
		    	#endif
                    #endif
                }
            }
         #if VRPMINTRACE
	 dumpTravelTime();
	 #endif
    }


    #ifdef OSRMCLIENT
    void getAllHints() {
        #ifdef DOSTATS
        Timer timer;
        #endif

        std::deque<std::string> hints;
        int total = original.size();
        int from, to;
        int i, j, k;

        for ( i = 0; ( i * 100 ) < total; i++ ) {
            from = i * 100;
            to = std::min ( ( i + 1 ) * 100 , total ) ;
            hints.clear();
            osrm->clear();

            for ( j = from; j < to ; j++ ) osrm->addViaPoint( original[j] );

            if ( osrm->getOsrmViaroute() and osrm->getOsrmHints( hints ) )
                for ( j = from, k = 0; j < to ; j++, k++ ) {
                    original[j].setHint( hints[k] );
                }
        }

        #ifdef DOSTATS
        STATS->addto( "TWC::getAllHints Cumultaive time:", timer.duration() );
        #endif

    }
    #endif


  public:
    /*!
     * \brief Assign the travel time matrix to TWC from Pg
     * \bug TODO needs to be tested when conected to the database
     * This method is specific for PostgreSQL integration. It receives a
     * ttime_t structure array containing the travel time matrix values passed
     * from the database and loads them into the problem and does some
     * additional needed computations.
     *
     * \param[in] ttimes The travel time data array from PosgreSQL
     * \param[in] count The count of entries in \b ttimes
     * \param[in] datanodes The data nodes Bucket previous loaded from PostgreSQL
     * \param[in] invalid The bucket of invalid nodes generated when load the data nodes.
     */
    void loadAndProcess_distance( ttime_t *ttimes, int count,
                                  const Bucket &datanodes, const Bucket &invalid ) {
	#ifdef DOVRPLOG
        DLOG( INFO ) << "POSTGRES: loadAndProcess_distance needs to be TESTED";
	#endif
        assert( datanodes.size() );
        original.clear();
        original = datanodes;

        #ifdef OSRMCLIENT
        getAllHints();
        #endif

        prepareTravelTime();

        for ( int i = 0; i < count; ++i ) {
            int from    = ttimes[i].from_id;
            int to      = ttimes[i].to_id;
            double time = ttimes[i].ttime;

            if ( invalid.hasId( from ) or invalid.hasId( to ) ) continue;

            int fromId = getNidFromId( from );
            int toId = getNidFromId( to );

            if ( fromId == -1 or toId == -1 ) continue;

            travel_Time[fromId][toId] = time;
        }

        twcij_calculate();
        assert ( original == datanodes );
        assert ( check_integrity() );
    }


    /*!
     * \brief Load the travel time matrix from a text file and process the results.
     *
     * Reads \b infile and loads it into the travel time matrix and populates
     * any missing entries we=ith an approx distance. It also computes the TWC
     * matrix.
     *
     * \todo Add description of file format.
     *
     * \param[in] infile The file name to load.
     * \param[in] datanodes The bucket of data nodes that has already been loaded.
     * \param[in] invalid A bucket of invalid nodes found in the data nodes.
     */
    void loadAndProcess_distance( std::string infile, const Bucket &datanodes,
                                  const Bucket &invalid ) {
        assert( datanodes.size() );
	#ifdef DOVRPLOG
        DLOG( INFO ) << "COMMANDLINE: loadAndProcess_distance";
	#endif

        original.clear();
        original = datanodes;
        int siz = original.size();

        std::ifstream in( infile.c_str() );
        std::string line;

        #ifdef OSRMCLIENT
        getAllHints();
	#endif

        prepareTravelTime();

        int fromId;
        int toId;
        int from, to;
        double time;
        int cnt = 0;

        while ( getline( in, line ) ) {
            cnt++;

            // skip comment lines
            if ( line[0] == '#' ) continue;

            std::istringstream buffer( line );
            buffer >> from;
            buffer >> to;
            buffer >> time;

            if ( invalid.hasId( from ) or invalid.hasId( to ) ) continue;

            fromId = getNidFromId( from );
            toId = getNidFromId( to );

            if ( fromId == -1 or toId == -1 ) continue;

            travel_Time[fromId][toId] = time;
        }

        in.close();

        twcij_calculate();
        assert ( original == datanodes );
        assert ( check_integrity() );
    }

    /*!
     * \brief Get a reference to the travel time matrix.
     *
     * \return A reference to the travel time matrix.
     */
    const std::vector<std::vector<double> > &TravelTime() {
        return travel_Time;
    }

    /*!
     * \brief Get the node id for a user id for a node.
     *
     * \param[in] id A user node identifier
     * \return The internal nid corresponding to id or -1 if not found.
     */
    long int getNidFromId( int id ) const {
        return original.getNidFromId( id );
    }


    static TWC<knode> *p_twc;
    static TWC<knode> *Instance() {
        if ( !p_twc ) // Only allow one instance of class to be generated.
            p_twc = new TWC<knode>;

        return p_twc;
    }






  private:
    // constructors
    TWC() {};
    TWC( const TWC & ) {};
    TWC &operator=( const TWC & ) {};



    /*!
     * \brief Fetch the time window compatibility value traveling from nids i to j.
     *
     * Return the compatibility of servicing customer toNid directly after
     * fromNod. Higher values represent better compatibility of the two time
     * windows being considered. And incompatible time windows will have
     * negative infinity.
     *
     * \param[in] i Nid of the from node.
     * \param[in] j Nid of the to node.
     * \return The time window compatibility of traveling directly \b fromNid to \b toNide.
     */
    double compat( int i, int j ) const {
        assert( i < original.size() and j < original.size() );
        return twcij[i][j];
    };


    /*!
     * \brief The earliest arrival time at \b nj from the latest departure from \b ni
     *
     * The earliest arrival time at \b nj, given that node \b nj is visted
     * directly after \b ni, and that we departed \b ni at the latest
     * possible time.
     *
     * \param[in] ni The node we departed from.
     * \param[in] nj The node we arrived at.
     * \return The earliest arrival time at \b nj
     */
    double ajli( const knode &ni, const knode &nj ) const {
        return ni.closes() + ni.getServiceTime() + TravelTime( ni, nj );
    }

    /*!
     * \brief The earliest arrival time at \b nj from the earliest departure from \b ni
     *
     * The earliest arrival time at \b nj, given that node \b nj is visted
     * directly after \b ni, and that we departed \b ni at the earliest
     * possible time.
     *
     * \param[in] ni The node we departed from.
     * \param[in] nj The node we arrived at.
     * \return The earliest arrival time at \b nj
     */
    double ajei( const knode &ni, const knode &nj ) const {
        return ni.opens() + ni.getServiceTime() + TravelTime( ni, nj );
    }


    /*!
     * \brief Compute TWC from node \b ni to node \b nj
     *
     * \param[in] ni From this node
     * \param[in] nj To this node
     * \return The TWC value traveling from node \b ni directly to \b nj
     */
    double twc_for_ij( const knode &ni, const knode &nj ) const {
        double result;
        int i = ni.getnid();
        int j = nj.getnid();

        if ( travel_Time[i][j] == -1 ) return  _MIN();

        if ( TravelTime( i, j ) == _MAX() ) return  _MIN();

        if ( ( nj.closes() - ajei( ni, nj ) ) > 0 ) {
            result = std::min ( ajli( ni, nj ) , nj.closes() )
                     - std::max ( ajei( ni, nj ) , nj.opens()  ) ;
        }
        else
            result = _MIN();

        return result;
    }


    double getTwcij( UID i, UID j ) const { //this one makes twcij dynamical
        if  ( travel_Time[i][j] == -1 ) {
            TravelTime( i, j );
            twcij[i][j] = twc_for_ij( original[i], original[j] );
        }

        return twcij[i][j];
    }

    double getTwcij( UID i, UID j, double time ) const {
        twcij[i][j] = twc_for_ij( original[i], original[j] );
        return twcij[i][j];
    }


    /* public functions That are id based */


    /*!
     * \brief Compute all TWC values and populate the TWC matrix.
     */
    void twcij_calculate() {
        assert ( original.size() == travel_Time.size() );
        twcij.resize( original.size() );

        for ( int i = 0; i < original.size(); i++ )
            twcij[i].resize( original.size() );


        for ( int i = 0; i < original.size(); i++ ) {
            for ( int j = i; j < original.size(); j++ ) {
                twcij[i][j] = twc_for_ij( original[i], original[j] );
                twcij[j][i] = twc_for_ij( original[j], original[i] );
            }
        }
    }

    /*!
     * \brief Check that a TWC matrix entry exists for all original nodes.
     */
    bool check_integrity() const {
        assert ( original.size() == twcij.size() );

        if ( original.size() != twcij.size() ) return false;

        for ( int i = 0; i < original.size(); i++ ) {
            assert ( twcij[i].size() == original.size() );

            if ( twcij[i].size() != original.size() ) return false;
        }

        return true;
    }



}; // end of class




template <class knode>
TWC<knode>  *TWC<knode>::p_twc = NULL;

#define twc TWC<Tweval>::Instance()



#endif
