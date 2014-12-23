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
#ifndef TWPATH_H
#define TWPATH_H

#include <deque>
#include <iostream>
#include <algorithm>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#include "node.h"
#include "twbucket.h"

/*!
 * \brief  Enumeration to indicate if evaluation has to be done
*/
enum E_Ret {
    OK        = 0,  ///< OK - the path was updated
    NO_CHANGE = 1,  ///< NO_CHANGE - there was no need to change the path
    INVALID   = 2   ///< INVALID - the requested move is not valid, no change was made
};

/*! \class Twpath
 * \brief Twpath class provides a path container that is auto evaluating.
 *
 * A path is an ordered sequence of nodes from start to end. This class
 * provides a self evaluating container for maintaining a path. The path has
 * attributes that are maintained with \ref Tweval along the path on each
 * node in the path such that the attributes on the last node in the path
 * reflect the totals for the whle path. Twpath also inherits all the non
 * evaluating methods as it extends \ref TwBucket.
 *
 * \sa \ref TwBucket a non evaluating container for nodes
 */
template <class knode>
class Twpath : public TwBucket<knode> {
    // ------------------------------------------------------------------
    // TwBucket has the non evaluating methods
    // Twpath has the evaluating methods &
    //      inherits the all the non evaluating methods
    // ------------------------------------------------------------------

  private:

    typedef unsigned long UID;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_reverse_iterator
    const_reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;

  public:
    // ------------------------------------------------------------------
    // methods from TwBucket class used in the evaluating functions
    // If a new evaluationg funtion uses a method of Twbucket not listed here
    //     add a new line with the correspondig name
    // ------------------------------------------------------------------
    using TwBucket<knode>::swap;
    using TwBucket<knode>::insert;
    using TwBucket<knode>::erase;
    using TwBucket<knode>::size;
    using TwBucket<knode>::path;


    /* ---------- operations within two  paths ------------------- */

    /*!
     * \brief Swap a node in path A with some node in path B
     *
     * Given two paths A and B and a position \b i in path A and position
     * \b j in path B, swap the node A[i] with B[j]. Like:
     *
     *      A.e_swap(i, maxCapacityA, B, j, maxCapacityB)
     *
     * \param[in] i Position of node in path A
     * \param[in] maxcap The maximum capacity of vehicle A
     * \param[in] rhs Vehicle B
     * \param[in] j Position of node in path B
     * \param[in] rhs_maxcap The maximum capacity of vehicle B
     * \return E_Ret to indicate status of the move request
     */
    E_Ret e_swap( UID i, double maxcap, Twpath<knode> &rhs, UID j,
                  double rhs_maxcap ) {
        assert ( i < size() and j < rhs.size() );

        if ( i < 0 or j<0 or i>size() - 1 or j > rhs.size() - 1 ) return INVALID;

        std::iter_swap( path.begin() + i, rhs.path.begin() + j );
        evaluate( i, maxcap );
        rhs.evaluate( j, rhs_maxcap );
        return OK;
    }



    // ----------  nodes handling within the same path --------------------

    /*!
     * \brief Evaluated: Move a node in a path to a new location.
     *
     * Move a node in a path to a new location and evaluate the resulting path.
     *
     * \param[in] fromi Position of node to move.
     * \param[in] toDest New position for the node in the path.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_move( UID fromi, UID toDest, double maxcapacity ) {
        assert ( fromi < size() and toDest < size() );

        if ( fromi < 0 or toDest<0 or fromi>size() - 1 or toDest > size() - 1 )
            return INVALID;

        if ( fromi == toDest ) return NO_CHANGE;

        if ( fromi < toDest ) {
            if ( toDest + 1 > path.size() )
                //I think this will never be executed
                path.push_back( path[fromi] );
            else
                insert( path[fromi], toDest + 1 );

            erase( fromi );
        }
        else {
            insert( path[fromi], toDest );
            erase( fromi + 1 );
        }

        fromi < toDest ? evaluate( fromi, maxcapacity ) : evaluate( toDest,
                maxcapacity );
        return OK;
    };


    /*!
     * \brief Evaluated: Resize a path by trucating it.
     *
     * This resize will only trucate a path as a fast way to remove nodes
     * from the end of the path. The resulting path is evaluated.
     *
     * \param[in] numb The number of to retain in the path.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_resize( UID numb, double maxcapacity ) {
        assert ( numb <= size() );

        if ( numb<0 or numb>size() ) return INVALID;

        path.resize( numb );
        //its reduced so the last node's value its not affected so no need of
        evalLast( maxcapacity ); //<--- can this one be avoided????
        return OK;
    };


    /*!
     * \brief Evaluated: Swap two nodes in the path.
     *
     * This method exchanges two nodes without a given path for the other
     * swapping them and then evaluating the resultant path.
     *
     * \param[in] i The position of the first node to swap.
     * \param[in] j The position of the second node to swap.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_swap( UID i, UID j, double maxcapacity ) {
        if ( i == j ) return NO_CHANGE;

        //if (i<0 or j<0 or i>size()-1 or j>size()-1) return INVALID;
        swap( i, j );
        i < j ? evaluate( i, maxcapacity ) : evaluate( j, maxcapacity );
        return OK;
    };


    /*!
     * \brief Evaluated: Move a range of nodes to a new position.
     *
     * Moves a range of nodes (i-j) to position k without reversing them
     * and evaluate the resultant path.
     *
     * \todo Probably more efficient with iterators
     * \param[in] i First node position in range to move.
     * \param[in] j Last node position in range to move.
     * \param[in] k Destination position to move range to.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_move( UID i, UID j, UID k, double maxcapacity ) {
        if ( ! ( i <= j and ( k > j or k < i ) ) ) return INVALID;

        if ( j > size() - 1 or k > size() ) return INVALID;

        // moving range to right of the range
        if ( k > j ) {
            // if the length of the range is larger than the distance
            // being moved it is faster to move the intervening nodes in
            // the opposite direction
            if ( j - i + 1 > k - j - 1 ) {
                return e_move( j + 1, k - 1, i, maxcapacity );
            }

            for ( int n = i, m = 0; n <= j; n++, m++ ) {
                knode temp = path[i];
                path.erase( path.begin() + i );
                path.insert( path.begin() + k - 1, temp );
            }
        }
        // moving range to left of the range
        else {
            // if the length of the range is larger than the distance
            // being moved it is faster to move the intervening nodes in
            // the opposite direction
            if ( j - i + 1 > i - k ) {
                return e_move( k, i - 1, j + 1, maxcapacity );
            }

            for ( int n = i, m = 0; n <= j; n++, m++ ) {
                knode temp = path[i + m];
                path.erase( path.begin() + i + m );
                path.insert( path.begin() + k + m, temp );
            }
        }

        //i < k ? path[i].evaluate(maxcapacity) : path[k].evaluate(maxcapacity);
        evaluate( maxcapacity );
        return OK;
    }


    /*!
     * \brief Evaluated: Move a range of nodes to a new position and reverse the order of the moved range of nodes.
     *
     * Moves a range of nodes (i-j) to position k and reverses the order of
     * the nodes in the range getting moved and evaluate the resultant path.
     *
     * \todo Probably more efficient with iterators
     * \param[in] i First node position in range to move.
     * \param[in] j Last node position in range to move.
     * \param[in] k Destination position to move range to.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_movereverse( UID i, UID j, int k, double maxcapacity ) {
        // path: 0 1 2 [3 4 5] 6 7 8 9
        // invalid moves are:
        //      rangeFrom > size-1 or rangeTo > size-1 or dest > size
        //      dest < 0 or to < 0 or from < 0 or to < from
        //      dest >= from and dest <= to+1
        if ( i > path.size() - 1 or j > path.size() - 1  or
             k > path.size() ) return INVALID;

        if ( i < 0 or j < 0 or k < 0 or j < i ) return INVALID;

        if ( k >= i and k <= j + 1 ) return INVALID;

        // moving range to right of the range
        if ( k > j ) {
            for ( int n = i, m = 1; n <= j; n++, m++ ) {
                knode temp = path[i];
                path.erase( path.begin() + i );
                path.insert( path.begin() + k - m, temp );
            }
        }
        // moving range to left of the range
        else {
            for ( int n = i; n <= j; n++ ) {
                knode temp = path[n];
                path.erase( path.begin() + n );
                path.insert( path.begin() + k, temp );
            }
        }

        evaluate( maxcapacity );
        return OK;
    }


    /*!
     * \brief Evaluated: Reverse the order of a range of nodes in the path.
     *
     * Reverse the order of the nodes in the range of positions from \b i to
     * \b j and evaluate the resulting path.
     *
     * \param[in] i First node position in range to reverse.
     * \param[in] j Last node position in range to reverse.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_reverse( UID i, UID j, double maxcapacity ) {
        assert ( i < size() and j < size() );

        if ( i<0 or j<0 or i >= path.size() or j >= path.size() )
            return INVALID;

        int m = i;
        int n = j;

        if ( i == j ) return NO_CHANGE;

        if ( i > j ) {
            m = j;
            n = i;
        }

        iterator itM = path.begin() + m;
        iterator itN = path.begin() + n;

        while ( itM < itN ) {
            std::iter_swap( itM, itN );
            itM++;
            itN--;
        }

        i < j ? evaluate( i, maxcapacity ) : evaluate( j, maxcapacity );
        return OK;
    };


    /*!
     * \brief Evaluated: Insert a node into an existing path.
     *
     * Insert a node into an existing path and evaluate the resultant path.
     *
     * \param[in] n The node to insert.
     * \param[in] at The position that the node should be inserted at.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_insert( const knode &n, UID at, double maxcapacity ) {
        assert ( at <= size() );

        if ( at < 0 or at > size() ) return INVALID;

        path.insert( path.begin() + at, n );
        evaluate( at, maxcapacity );
        return OK;
    };


    /*!
     * \brief Evaluated: Append a node to an existing path.
     *
     * Append a node to an existing path and evaluate the resultant path.
     *
     * \param[in] n The node to be appended.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_push_back( const knode &n, double maxcapacity ) {
        path.push_back( n );
        evalLast( maxcapacity );
        return OK;
    };


    /*!
     * \brief Evaluated: Remove a node from a path.
     *
     * Remove the node at the given position from a path and evaluate
     * the resultant path.
     *
     * \param[in] i The position of the node to be removed.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     * \return Status of whether or not the move was made.
     */
    E_Ret e_remove ( UID i, double maxcapacity ) {
        assert ( i < size() );

        if ( i<0 or i>size() - 1 ) return INVALID;

        path.erase( path.begin() + i );
        evaluate( i, maxcapacity );
        return OK;
    };

    /* --------------   EVALUATION  --------------------------- */


    /*!
     * \brief Evaluated: Evaluate the whole path from the start.
     *
     * Path evaluation is done incrementally from a change to the
     * end of the path and intermediate values are cached on each node.
     * So if we change the path at position 10, it only needs to be evaluated
     * from that position forward. Thise evaluates the whole path.
     *
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     */
    void evaluate( double maxcapacity ) {
        assert ( size() > 0 );
        evaluate( 0, maxcapacity );
    };


    /*!
     * \brief Evaluated: Evaluate a path from the given position.
     *
     * Path evaluation is done incrementally from a change to the
     * end of the path and intermediate values are cached on each node.
     * So if we change the path at position 10, it only needs to be evaluated
     * from that position forward.
     *
     * \param[in] from The starting position in the path for evaluation to
     * the end of the path.
     * \param[in] maxcapacity The maximum capacity of vehicle for this path.
     */
    void evaluate( UID from, double maxcapacity ) {
        // the equal just in case the last operation was erase
        assert ( from <= size() );

        if ( from >= path.size() ) from = size() - 1;

        iterator it = path.begin() + from;

        while ( it != path.end() ) {
            if ( it == path.begin() ) it->evaluate( maxcapacity );
            else it->evaluate( *( it - 1 ), maxcapacity );

            it++;
        }

    };


    void evaluateOsrm( const std::string &osrmBaseUrl ) {
        assert ( size() > 0 );
        evaluateOsrm( 0, osrmBaseUrl );
    }


    void evaluateOsrm( UID from, const std::string &osrmBaseUrl ) {
        // the equal just in case the last operation was erase
        assert ( from <= size() );

        if ( from >= path.size() ) from = size() - 1;

        iterator it = path.begin() + from;

        while ( it != path.end() ) {
            if ( it == path.begin() ) it->evaluateOsrm();
            else it->evaluateOsrm( *( it - 1 ) , osrmBaseUrl );

            ++it;
        }
    };

    bool isOsrmTtimeValid() const {
        return path[path.size() - 1].isOsrmTtimeValid();
    };

    double getTotTravelTimeOsrm() const {
        return path[path.size() - 1].getTotTravelTimeOsrm();
    }


    bool feasable() const {
        return ( path[path.size() - 1].feasable() );
    };

    void evalLast( double maxcapacity ) {
        assert ( size() > 0 );
        evaluate( path.size() - 1, maxcapacity );
    };

    bool operator ==( const Twpath<knode> &other ) {
        if ( size() != other.size() ) return false;

        iterator it = path.begin();
        iterator ito = other.path.begin();

        while ( it != path.end() ) {
            if ( it->getnid() != ito->getnid() ) return false;

            ito++; it++;
        }
    }

    void dumpeval() const {
        for ( int i = 0; i < path.size(); i++ )
            path[i].dumpeval();
    }


    Twpath<knode> &operator =( const TwBucket<knode> &other ) {
        TwBucket<knode>::operator = ( other );
        return *this;
    }

    Twpath<knode> &operator -=( const TwBucket<knode> &other ) {
        assert( std::string("Set operation not allowed on derived class of Twpath, Overload -= if this is required")
                == std::string(" "));
    }
    Twpath<knode> &operator *=( const TwBucket<knode> &other ) {
        assert( std::string("Set operation not allowed on derived class of Twpath, Overload *= if this is required")
                == std::string(" "));
    }
    Twpath<knode> &operator +=( const TwBucket<knode> &other ) {
        assert( std::string("Set operation not allowed on derived class of Twpath, Overload += if this is required")
                == std::string(" "));
	return *this;
    }
    ////// (double underscore) (considers  Dumps;)

    bool createsViolation( UID from, double maxcapacity ) {
        #ifdef TESTED
        DLOG( INFO ) << "Entering twpath::createsViolation";
        #endif
        assert ( from <= size() ); //the equal just in case the last operation was erase

        if ( from >= path.size() ) from = size() - 1;

        iterator it = path.begin() + from;

        while ( it != path.end() ) {
            if ( it == path.begin() ) it->evaluate( maxcapacity );
            else {
                it->evaluate( *( it - 1 ), maxcapacity );

                if ( not it->feasable() ) return true;
            }

            if ( it->isDump() ) break;

            it++;
        }

        return false;
    };


    // doesnt insert if it creates a CV or TWV violation
    // dosnt move dumps
    bool e__insert( const knode &n, UID at, double maxcapacity ) {
        #ifdef TESTED
        DLOG( INFO ) << "Entering twpath::e__insert";
        #endif
        assert ( at <= size() );
        assert ( at > 0 );
        evaluate ( at, maxcapacity );
        //if ( not path[size()-1].feasable() ) return false;
        assert ( path[size() - 1].feasable() );
        path.insert( path.begin() + at, n );

        if ( createsViolation( at, maxcapacity ) ) {
            erase( at );

            if ( not createsViolation( at, maxcapacity ) ) return false;

            assert ( true == false );
        }

        assert ( path[size() - 1].feasable() );
        return true;
    };

    bool e__adjustDumpsToMaxCapacity( int currentPos, const knode &dumpS,
                                      double maxcapacity ) {
        //TODO convert to iterators

        #ifdef TESTED
        DLOG( INFO ) << "Entering twpath::e__adjustDumpsToMaxCapacity";
        #endif
        knode dumpSite = dumpS;
        int i = currentPos;

        while ( i < path.size() ) {
            if ( path[i].isDump() ) erase( i );
            else i++;
        };

        evaluate( currentPos, maxcapacity ); //make sure everything is evaluated

        if ( path[size() - 1].feasable() ) return true; // no need to add a dump

        if (  path[size() - 1].gettwvTot() ) return
                false; // without dumps its unfeasable

        //the path is dumpless from the currentpos
        //add dumps because of CV
        while ( path[size() - 1].getcvTot() )  {
            //cycle until we find the first non CV
            for ( i = path.size() - 1; i >= currentPos - 1 and path[i].getcvTot(); i-- ) {};

            insert( dumpSite, i + 1 ); // the dump should be after pos i

            evaluate( i, maxcapacity ); //reevaluate the rest of the route

            #ifdef TESTED
            DLOG( INFO ) << "Entering twpath::e__adjustDumpsToMaxCapacity: inserted a dump";

            dumpeval();

            #endif

            // dont bother going to what we had before
            // added a dump and  is no cv and no twv
            if (  path[size() - 1].feasable() ) return true;

            // added a dump and created a twv, so why bother adding another dump
            if (  path[size() - 1].gettwvTot() ) return false;
        };

        return  path[size() - 1].feasable() ;
    };

};


#endif


