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

#include "node.h"
#include "twpath.h"
#include "pg_types_vrp.h"

/*! \class TWC
 * \brief Class TWC (Time Window Compatibility) provides tools for working with Twpath objects.
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
 */
template <class knode> class TWC {
  private:
    typedef TwBucket<knode> Bucket;
    typedef unsigned long int UID;
    typedef unsigned long int POS;


    static TwBucket<knode> original;
    static std::vector<std::vector<double> > twcij;
    std::vector<std::vector<double> > travel_Time;

    inline double _MIN() const { return -std::numeric_limits<double>::max();};
    inline double _MAX() const { return std::numeric_limits<double>::max();};


  public:

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


    /*!
     * \brief Select all nodes in a bucket from which you can not reach node to.
     *
     * Given a bucket of nodes, return all the nodes from which you can not
     * reach a given node because of time window compatibility.
     *
     * For example, if a node has a time window closes at 0800 then that node
     * is unreachable from all nodes that have a time window that opens after
     * that time. There are also other factors to consider like travel time
     * between nodes, waiting time for early arrival, and service times at
     * nodes.
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
     * from the input node because of time window violations.
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
     * node is \b to without createing time window violations.
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
     * \brief 
     *
     */
    TwBucket<knode> getIncompatible( const TwBucket<knode> &nodes, UID to ) const {
        assert( to < original.size() );
        Bucket incompatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( not isCompatibleIJ( nodes[i].getnid(), to ) )
                incompatible.push_back( nodes[i] );

        return incompatible;
    }

    TwBucket<knode> getIncompatible( UID from,
                                     const TwBucket<knode> &nodes ) const {
        assert( from < original.size() );
        Bucket incompatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( not isCompatibleIJ( from, nodes[i].getnid() ) )
                incompatible.push_back( nodes[i] );

        return incompatible;
    }

    TwBucket<knode> getCompatible( const TwBucket<knode> &nodes, UID to ) const {
        assert( to < original.size() );
        Bucket compatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( isCompatibleIJ( nodes[i].getnid(), to ) )
                compatible.push_back( nodes[i] );

        return compatible;
    }

    TwBucket<knode> getCompatible( UID from, const TwBucket<knode> &nodes ) const {
        assert( from < original.size() );
        Bucket compatible;

        for ( int i = 0; i < nodes.size(); i++ )
            if ( isCompatibleIJ( from, nodes[i].getnid() ) )
                compatible.push_back( nodes[i] );

        return compatible;
    }


    double travelTime( UID from, UID to ) const {
        assert( from < original.size() and to < original.size() );
        return travel_Time[from][to];
    }

    double travelTime( const knode &from, const knode &to ) const {
        return travelTime( from.getnid(), to.getnid() );
    }

    double compatibleIJ( UID fromNid, UID toNid ) const {
        assert( fromNid < original.size() and toNid < original.size() );
        return  twcij[fromNid][toNid] ;
    }

    const knode &node( UID i ) const {
        assert( i < original.size() );
        return original[i];
    };

    const knode &getNode( UID at ) const {
        assert( at < original.size() );
        return original[at];
    };

    //state
    bool isCompatibleIJ( UID fromNid, UID toNid ) const {
        assert( fromNid < original.size() and toNid < original.size() );
        return not ( twcij[fromNid][toNid]  == _MIN() );
    }

    bool isReachableIJ( UID fromNid, UID toNid ) const {
        assert( fromNid < original.size() and toNid < original.size() );
        return not ( travel_Time[fromNid][toNid]  == _MIN() );
    }


    bool isCompatibleIAJ( UID fromNid, UID middleNid, UID toNid ) const {
        assert( fromNid < original.size() and middleNid < original.size()
                and toNid < original.size() );
        return isCompatibleIJ( fromNid, middleNid )
               and isCompatibleIJ( middleNid, toNid );
    }


    bool isCompatibleIAJ( const knode &from, const knode &middle,
                          const knode &to ) const {
        return isCompatibleIAJ( from.getnid(), middle.getnid() , to.getnid() );
    }






    //  The best or the worses

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



    int  getBestCompatible( UID fromNid, const Bucket &nodes ) const {
        assert( fromNid < original.size() );
        int bestId;
        int toId;

        if ( nodes.empty() ) return -1;

        bestId = nodes[0].getnid();

        for ( int j = 0; j < nodes.size(); j++ ) {
            toId = nodes[j].getnid();

            if ( twcij[fromNid][toId] > twcij[fromNid][bestId] ) {
                bestId = toId;
            }
        }

        if ( compat( fromNid, bestId ) != _MIN() ) return bestId;
        else return -1;
    }




    double ec2( UID at, const Bucket &nodes ) {
        assert( at < original.size() );
        double ec2_tot = 0;

        for ( int j = 0; j < nodes.size(); j++ ) {
            if ( not ( twcij[at][j]  == _MIN() ) ) ec2_tot += twcij[at][j];

            if ( not ( twcij[j][at]  == _MIN() ) ) ec2_tot += twcij[j][at];
        };

        if ( twcij[at][at] == _MIN() ) ec2_tot -= twcij[at][at];

        return ec2_tot;
    }


    //counting
    int countIncompatibleFrom( int at, const Bucket &nodes ) {
        assert( at < original.size() );
        int count = 0;

        for ( int j = 0; j < nodes.size(); j++ ) {
            if ( twcij[at][j]  == _MIN() ) count++;
        }

        return count;
    }

    int countIncompatibleTo( int at, const Bucket &nodes ) {
        int count = 0;

        for ( int j = 0; j < nodes.size(); j++ ) {
            if ( twcij[j][at]  == _MIN() ) count++;
        }

        return count;
    }



    /*    DUMPS   */
    void dump() const  {
        assert( original.size() );
        dump( original );
    }


    void dump( const Bucket &nodes ) const  {
        assert( original.size() );
        dumpCompatability( original );
        dumpTravelTime( original );
    }

    void dumpCompatability() const  {
        assert( original.size() );
        dumpCompatability( original );
    }

    void dumpCompatability( const Bucket &nodes ) const  {
        assert( nodes.size() );
        std::cout.precision( 8 );
        std::cout << "COMPATABILITY TABLE \n\t";

        for ( int i = 0; i < nodes.size(); i++ )
            std::cout << "nid " << nodes[i].getnid() << "\t";

        std::cout << "\n\t";

        for ( int i = 0; i < nodes.size(); i++ )
            std::cout << "id " << nodes[i].getid() << "\t";

        std::cout << "\n";

        for ( int i = 0; i < nodes.size(); i++ ) {
            std::cout << nodes[i].getnid() << "=" << nodes[i].getid() << "\t";

            for ( int j = 0; j < nodes.size(); j++ ) {
                if ( twcij[i][j] !=  -std::numeric_limits<double>::max() ) std::cout <<
                            twcij[i][j] << "\t";
                else std::cout << "--\t";
            }

            std::cout << "\n";
        }
    }

    void dumpTravelTime() const {
        assert( original.size() );
        dumpTravelTime( original );
    }

    void dumpTravelTime( const Bucket &nodes ) const {
        assert( nodes.size() );
        std::cout << "\n\n\n\nTRAVEL TIME TABLE \n\t";

        std::cout.precision( 2 );

        for ( int i = 0; i < nodes.size(); i++ )
            std::cout << "nid " << nodes[i].getnid() << "\t";

        std::cout << "\n\t";

        for ( int i = 0; i < nodes.size(); i++ )
            std::cout << "id " << nodes[i].getid() << "\t";

        std::cout << "\n";

        for ( int i = 0; i < nodes.size(); i++ ) {
            std::cout << nodes[i].getnid() << "=" << nodes[i].getid() << "\t";

            for ( int j = 0; j < nodes.size(); j++ ) {
                if ( travel_Time[i][j] !=  std::numeric_limits<double>::max() ) std::cout <<
                            travel_Time[i][j] << "\t";
                else std::cout << "--\t";
            }

            std::cout << "\n";
        }
    }


    void dumpCompatible3() const  {
        assert( original.size() );
        dumpCompatible( original );
    }

    void dumpCompatible3( const Bucket &nodes ) const {
        assert( nodes.size() );

        for ( int i = 0; i < nodes.size(); i++ ) {
            for ( int j = 0; j < nodes.size(); j++ ) {
                for ( int k = 0; k < nodes.size(); k++ ) {
                    std::cout << "\t ( " << nodes[i].getnid() << " , "
                              << nodes[j].getnid() << " , "
                              << nodes[k].getnid() << ") = "
                              << ( isCompatibleIAJ( i, j, k ) ? "COMP" : "not" );
                }

                std::cout << "\n";
            }
        }
    }




    //go back to CALCULATED state
    void recreateCompatible( UID nid ) {
        assert( nid < original.size() );

        for ( int j = 0; j < twcij.size(); j++ ) {
            twcij[nid][j] = twc_for_ij( original[nid], original[j] );
            twcij[j][nid] = twc_for_ij( original[j], original[nid] );
        }
    }

    // Functions to adjust compatability depending on problem
    void recreateTravelTime( UID nid ) {
        assert( "needs to be re-read from file" == "" );
    }


    void setIncompatible( UID fromNid, UID toNid ) {
        assert( fromNid < original.size() and toNid < original.size() );
        twcij[fromNid][toNid] = _MIN();
    }




    void setIncompatible( UID nid, const Bucket &nodes ) {
        assert( nid < original.size() );

        for ( int j = 0; j < nodes.size(); j++ )
            twcij[nid][nodes[j].getnid()] =  _MIN();
    }


    void setIncompatible( const Bucket &nodes, UID &nid ) {
        assert( nid < original.size() );

        for ( int i = 0; i < nodes.size(); i++ )
            twcij[nodes[i].getnid()][nid] =  _MIN();
    }

    void setUnreachable( UID fromNid, UID toNid ) {
        assert( fromNid < original.size() and toNid < original.size() );
        travel_Time[fromNid][toNid] = _MAX();
    }

    void setUnreachable( UID nid, const Bucket &nodes ) {
        assert( nid < original.size() );

        for ( int j = 0; j < nodes.size(); j++ )
            travel_Time[nid][nodes[j].getnid()] =  _MAX();
    }

    void setUnreachable( const Bucket &nodes, UID &nid ) {
        assert( nid < original.size() );

        for ( int i = 0; i < nodes.size(); i++ )
            travel_Time[nodes[i].getnid()][nid] =  _MAX();
    }


    int setNodes( Bucket _original ) {
        original.clear();
        original = _original;
        twcij_calculate();
        assert ( original == _original );
        assert ( check_integrity() );
    }


    void loadAndProcess_distance( ttime_t *ttimes, int count,
                                  const Bucket &datanodes, const Bucket &invalid ) {
        assert( datanodes.size() );
        original.clear();
        original = datanodes;
        int siz = original.size();

        travel_Time.resize( siz );

        for ( int i = 0; i < siz; i++ )
            travel_Time[i].resize( siz );

        //travel_Time default value is 250m/min
        for ( int i = 0; i < siz; i++ )
            for ( int j = i; j < siz; j++ ) {
                if ( i == j ) travel_Time[i][i] = 0;
                else travel_Time[i][j] = travel_Time[j][i]
                                             = original[i].distance( original[j] ) / 250;
            }

        std::cout << siz << "<---- size\n";

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


    double getAverageTime( const Bucket &from, const knode &to ) const {
        assert( to.getnid() < original.size() );
        double time = 0;
        int j = to.getnid();

        for ( int i = 0; i < from.size(); i++ ) {
            time += travelTime( from[i].getnid() , j ) ;
        }

        time = time / from.size();
        return time;
    }

    double getAverageTime( const knode &from, const Bucket &to ) const {
        assert( from.getnid() < original.size() );
        double time = 0;
        int j = from.getnid();

        for ( int i = 0; i < to.size(); i++ )
            time += travel_Time[j][ to[i].getnid() ];

        time = time / to.size();
        return time;
    }

    void settCC ( const knode &C, const Bucket &picks ) {
        int pos = C.getnid();
        travel_Time[pos][pos] = getAverageTime( C, picks );
    }

    bool sameStreet( int i, int j ) {
        return original[i].sameStreet( original[j] );
    }

    double gradient( int i, int j ) {
        return original[i].gradient( original[j] );
    }

    void loadAndProcess_distance( std::string infile, const Bucket &datanodes,
                                  const Bucket &invalid ) {
        assert( datanodes.size() );
        original.clear();
        original = datanodes;
        int siz = original.size();

        std::ifstream in( infile.c_str() );
        std::string line;
        int fromId;
        int toId;
        travel_Time.resize( siz );

        for ( int i = 0; i < siz; i++ )
            travel_Time[i].resize( siz );

        //travel_Time default value is 250m/min
        for ( int i = 0; i < siz; i++ )
            for ( int j = i; j < siz; j++ ) {
                if ( i == j ) travel_Time[i][i] = 0;
                else {
                    travel_Time[i][j] = travel_Time[j][i]
                                        = original[i].distance( original[j] ) / 250;

                    if ( not sameStreet( i, j ) )
                        travel_Time[i][j] = travel_Time[i][j] *
                                            ( std::abs( std::sin( gradient( i, j ) ) )
                                              + std::abs( std::cos( gradient( i, j ) ) )  );
                }
            }

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

    const std::vector<std::vector<double> > &TravelTime() { return travel_Time;}

    long int getNidFromId( int id ) const { return original.getNidFromId( id ); }


    // constructors
    TWC() {};

    /* private are indexed */
  private:
    double compat( int i, int j ) const {
        assert( i < original.size() and j < original.size() );
        return twcij[i][j];
    };

    double ajli( const knode &ni, const knode &nj ) {
        return ni.closes() + ni.getservicetime() + travelTime( ni, nj );
    }

    double ajei( const knode &ni, const knode &nj ) {
        return ni.opens() + ni.getservicetime() + travelTime( ni, nj );
    }


    double twc_for_ij( const knode &ni, const knode &nj ) {
        double result;

        if ( travelTime( ni, nj ) == _MAX() ) result = _MIN();
        else if ( ( nj.closes() - ajei( ni, nj ) ) > 0 ) {
            result = std::min ( ajli( ni, nj ) , nj.closes() )
                     - std::max ( ajei( ni, nj ) , nj.opens()  ) ;
        }
        else result = _MIN();

        return result;
    }



    /* public functions That are id based */
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

    bool check_integrity() const {
        assert ( original.size() == twcij.size() );

        for ( int i = 0; i < original.size(); i++ ) {
            assert ( twcij[i].size() == original.size() );
        }

        return true;
    }



};

template <class knode>
TwBucket<knode> TWC<knode>::original;

template <class knode>
std::vector<std::vector<double> >  TWC<knode>::twcij;
/*
template <class knode>
std::vector<std::vector<double> >  TWC<knode>::travel_Time;
*/


#endif
