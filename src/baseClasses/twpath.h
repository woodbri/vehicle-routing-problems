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
#ifndef SRC_BASECLASSES_TWPATH_H_
#define SRC_BASECLASSES_TWPATH_H_

#include <deque>
#include <iostream>
#include <algorithm>

#ifdef DOVRPLOG
#include "./logger.h"
#endif

#include "./node.h"
#include "./twbucket.h"

#ifdef OSRMCLIENT
#include "./osrmclient.h"
#endif

/*! \class Twpath
 * \brief Twpath class members are auto evaluating.
 * 
 * The intention for this class is to have GENERAL functions that can
 *   be used in different types of problems. Therefore is strongly
 *   recommended that especific problem functions be coded in the
 *   problems vehicle
 *
 * \warning prefix: e_ performs the operation and evaluates
 * \warning prefix: ef_ performs the operation only if the resulting
 *   path is feasable
 * \warning prefix: e__ performs the operation on especific problems and
 *   eventually shall be removed
 * 
 * \note All members return \b true when the operation is succesfull
 * 
 * Twpath also inherits all the non evaluating methods of \ref TwBucket.
 *  
 * A path is an ordered sequence of nodes from starting site to ending site.
 * The problem will define which type of nodes belongs to the twpath and
 * which shall be outside twpath.
 *
 * \sa \ref TwBucket a non evaluating container for nodes
 */
template <class knode>
class Twpath : public TwBucket<knode> {
 private:
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
  using TwBucket<knode>::push_back;
  using TwBucket<knode>::feasable;
  using TwBucket<knode>::twvTot;
  using TwBucket<knode>::cvTot;

  /*! @name deque like functions 
    
    \returns True if the operation was performed
    \warning Assertions are performed for out of range operations
    \warning no feasability nor time window or capacity violations 
      checks are performed
    \todo TODO more deque like functions here 
  */
  ///@{
  /*! \brief Evaluated: Insert a node into an existing path.
   *
   * \param[in] node The node to insert.
   * \param[in] at The position that the node should be inserted.
   * \param[in] maxcapacity The maximum capacity of vehicle for this path.
   */
  bool e_insert( const knode &node, POS at, double maxcapacity ) {
    assert (at <= size());
    if (not insert(node , at) )
      return false;
    evaluate(at, maxcapacity);
    return true;
  }


  /*! \brief Evaluated: push_back a node to the path.
   *
   * \param[in] node to be push_back.
   * \param[in] maxcapacity of vehicle for this path.
   */
  bool e_push_back(const knode &node, double maxcapacity) {
    if (not push_back(node))
      return false;
    evalLast(maxcapacity);
    return true;
  }

  /*!  * \brief Evaluated: erase a node from the path.
   *
   * \param[in] pos to be erased.
   * \param[in] maxcapacity of vehicle for this path.
   */
  bool e_erase(POS pos, double maxcapacity) {
    assert (pos < size());

    if (not erase(pos))
      return false;

    evaluate(pos, maxcapacity);
    return true;
  }
 
  /*!  * \brief Evaluated: erase a node from the path.
   *
   * \param[in] node to be erased.
   * \param[in] maxcapacity of vehicle for this path.
   */
  bool e_erase(const knode &node, double maxcapacity) {
    if ( !hasNid(node) )
      return false;
    int atPos = pos(node);
    assert (atPos < size());

    if (not erase(atPos))
      return false;

    evaluate(atPos, maxcapacity);
    return true;
  }
  ///@}


  /*! @name Evaluation 
   * Path evaluation is done incrementally: from a given position to the
   * end of the path, and intermediate values are cached on each node.
   * So, for example, changing the path at position 100:
   * the evaluation function should be called as
   *  \c evaluate(100,maxcapacity)
   * and from that position to the end of the path will be evaluated.
   * None of the "unaffected" positions get reevaluated
  */
  ///@{
  /*! \brief Evaluated: Evaluate the whole path from the start.
   *
   * \param[in] maxcapacity The maximum capacity of vehicle for this path.
   */
  void evaluate(double maxcapacity) {
    assert(size() > 0);
    evaluate(0, maxcapacity);
  }

  /*! \brief Evaluated: Evaluate a path from the given position.
   *
   * \param[in] from The starting position in the path for evaluation to
   * the end of the path.
   * \param[in] maxcapacity The maximum capacity of vehicle for this path.
   */
  void evaluate( UID from, double maxcapacity ) {
    // the equal just in case the last operation was erase
    assert(from <= size());

    if ( from >= path.size() ) from = size() - 1;

    iterator node = path.begin() + from;

    while (node != path.end()) {
      if (node == path.begin()) node->evaluate(maxcapacity);
      else node->evaluate(*(node - 1), maxcapacity);

      node++;
    }

  }
  
  /*! \brief Evaluated: Evaluate the last node of the path.
   *
   * \param[in] maxcapacity The maximum capacity of vehicle for this path.
   */
  void evalLast( double maxcapacity ) {
    assert(size() > 0);
    evaluate( path.size() - 1, maxcapacity );
  }

  /*! \brief Evaluates the whole path with OSRM

   \warning all values in twc->time_Table (2 dim) involved on the path
      are updated to the structure of the path.
      (this especially affects the values on 2 way streets)
   \note the benefit of modifying those values is that the next
      non OSRM evaluation gives the same evaluating results
 */
  #ifdef OSRMCLIENT
  void evaluateOsrm(double maxcapacity) {
    assert(size() > 0);
    assert(osrmi->getUse());
    if (!osrmi->getConnection()) {
      #ifdef VRPMINTRACE
      DLOG(INFO)<<"OSRM connection not found: using normal evaluation";
      #endif
      evaluate(0, maxcapacity);
      return;
    };
    osrmi->clear();
    for ( iterator node = path.begin(); node != path.end(); node++) {
      if ( node == path.begin() ) {
        node->evaluate(maxcapacity);
        osrmi->addViaPoint(*node);
      } else {
        node->evaluateOsrm(*(node - 1), maxcapacity);
      }
    }
  }
  #endif
  ///@}


  /*! @name operators */
  ///@{
  bool operator ==( const Twpath<knode> &other ) {
    if ( size() != other.size() ) return false;

    iterator it = path.begin();
    iterator ito = other.path.begin();

    while ( it != path.end() ) {
      if ( it->getnid() != ito->getnid() ) return false;

      ito++; it++;
    }
  }

  Twpath<knode> &operator =( const TwBucket<knode> &other ) {
    TwBucket<knode>::operator = ( other );
    return *this;
  }
  ///@}

  #ifdef DOVRPLOG
  void dumpeval() const {
    for ( int i = 0; i < path.size(); i++ )
      path[i].dumpeval();
  }
  #endif

  
  

  /*! @name Very Uruguay project especific for trash problem with moving dumps
    \warning be carfull when using in other problems
    \todo TODO move to trash problem with moving dump's vehicle
  */
  ///@{
  bool e__createsViolation( UID from, double maxcapacity ) {
#ifdef TESTED
    DLOG( INFO ) << "Entering twpath::e__createsViolation";
#endif
    assert (from <= size()); //the equal just in case the last operation was erase

    if ( from >= path.size() ) from = size() - 1;

    iterator it = path.begin() + from;

    while ( it != path.end() ) {
      if ( it == path.begin() ) {
        it->evaluate(maxcapacity);
      } else {
        it->evaluate(*( it - 1 ), maxcapacity);

        if ( not it->feasable() ) return true;
      }

      if ( it->isDump() ) break; // TODO why not continue???

      it++;
    }

    return false;
  }


  // doesnt insert if it creates a CV or TWV violation
  // dosnt move dumps
  bool e__insert( const knode &n, UID at, double maxcapacity ) {
#ifdef TESTED
    DLOG( INFO ) << "Entering twpath::e__insert";
#endif
    assert(at <= size());
    assert(at > 0);
    evaluate(at, maxcapacity);
    //if ( not path[size()-1].feasable() ) return false;
    assert(feasable());
    path.insert(path.begin() + at, n);

    if ( e__createsViolation(at, maxcapacity) ) {
      erase(at);
      if (not e__createsViolation(at, maxcapacity)) return false;
      assert(std::string("rollback went wrong") == std::string(" "));
    }

    assert (feasable());
    return true;
  }

  bool e__adjustDumpsToMaxCapacity( int currentPos, const knode &dumpS,
                                    double maxcapacity ) {
    // TODO move to vehicle of trashwithmovingdumps

#ifdef TESTED
    DLOG( INFO ) << "Entering twpath::e__adjustDumpsToMaxCapacity";
#endif
    knode dumpSite = dumpS;
    int i = currentPos;

    while ( i < path.size() ) {
      if ( path[i].isDump() ) erase(i);
      else i++;
    }

    evaluate(currentPos, maxcapacity); //make sure everything is evaluated

    if ( feasable() ) return true; // no need to add a dump

    if ( twvTot() != 0 ) return false; // without dumps its unfeasable

    //the path is dumpless from the currentpos
    //add dumps because of CV
    while ( cvTot() != 0 )  {
      //cycle until we find the first non CV
      for ( i = path.size() - 1; i >= currentPos - 1 and path[i].cvTot(); i-- ) {}

      insert(dumpSite, i + 1); // the dump should be after pos i

      evaluate(i, maxcapacity); //reevaluate the rest of the route

#ifdef TESTED
      DLOG( INFO ) << "Entering twpath::e__adjustDumpsToMaxCapacity: inserted a dump";

#endif

      // dont bother going to what we had before
      // added a dump and  is no cv and no twv
      if (  feasable() ) return true;

      // added a dump and created a twv, so why bother adding another dump
      if (  twvTot() ) return false;  // no roll back
    }

    return  feasable() ;
  }
  ///@}



#if 0
  /*! @name  operations within two  paths
      \todo TODO fix return to be true when operation is performed
  */
  ///@{
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
   * \return true when the swap was performed
   */
  bool e_swap(POS i, double maxcap, Twpath<knode> &rhs, POS j,
                double rhs_maxcap) {
    assert(i < size() and j < rhs.size());

    if ( i < 0 or j<0 or i>size() - 1 or j > rhs.size() - 1 ) return false;

    std::iter_swap(path.begin() + i, rhs.path.begin() + j);
    evaluate( i, maxcap );
    rhs.evaluate( j, rhs_maxcap );
    return true;
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
  bool e_move( POS fromi, POS toDest, double maxcapacity ) {
    assert(fromi < size() and toDest < size());
    if ( fromi < 0 or toDest<0 or fromi>size() - 1 or toDest > size() - 1 )
      return false;

    assert(fromi != toDest);
    if ( fromi == toDest ) return false;

    if ( fromi < toDest ) {
      if ( toDest + 1 > path.size() )
        //I think this will never be executed
        path.push_back( path[fromi] );
      else
        insert(path[fromi], toDest + 1);

      erase(fromi);
    } else {
      insert(path[fromi], toDest);
      erase(fromi + 1 );
    }

    fromi < toDest ? evaluate(fromi, maxcapacity) 
                    : evaluate(toDest, maxcapacity);
    return true;
  }


  /*! \brief Evaluated: Resize a path by trucating it.
   *
   * This resize will only trucate a path as a fast way to remove nodes
   * from the end of the path. The resulting path is evaluated.
   *
   * \param[in] numb The number of to retain in the path.
   * \param[in] maxcapacity The maximum capacity of vehicle for this path.
   * \return Status of whether or not the move was made.
   */
  bool e_resize(UINT newSize, double maxcapacity) {
    assert ( newSize <= size() );
    if ( newSize<0 or newSize>size() ) return false;

    path.resize(newSize);
    // its reduced so the last node's value its not affected so no need of
    evalLast(maxcapacity); // TODO <--- can this one be avoided????
    return true;
  }


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
  bool e_swap(UID i, UID j, double maxcapacity) {
    if (i == j) return false;

    swap(i, j);
    i < j ? evaluate(i, maxcapacity) : evaluate(j, maxcapacity);
    return true;
  }


  /*! \brief Evaluated: Move a range of nodes to a new position.
   *
   * Moves a range of nodes (i-j) to position k without
   * and evaluate the resultant path.
   *
   * \todo Probably more efficient with iterators
   * \param[in] i First node position in range to move.
   * \param[in] j Last node position in range to move.
   * \param[in] k Destination position to move range to.
   * \param[in] maxcapacity The maximum capacity of vehicle for this path.
   * \return Status of whether or not the move was made.
   */
  bool e_move(UID i, UID j, UID k, double maxcapacity) {
    if ( !(i <= j and (k > j or k < i ))) return false;

    if ( j > size() - 1 or k > size() ) return false;

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

    // TODO i < k ? path[i].evaluate(maxcapacity) : path[k].evaluate(maxcapacity);
    evaluate(maxcapacity);
    return true;
  }


  /*! \brief Evaluated: Move a range of nodes to a new position and reverse the order of the moved range of nodes.
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
  bool e_movereverse( UID i, UID j, int k, double maxcapacity ) {
    // path: 0 1 2 [3 4 5] 6 7 8 9
    // invalid moves are:
    //      rangeFrom > size-1 or rangeTo > size-1 or dest > size
    //      dest < 0 or to < 0 or from < 0 or to < from
    //      dest >= from and dest <= to+1
    if ( i > path.size() - 1 or j > path.size() - 1  or
         k > path.size() ) return false;

    if ( i < 0 or j < 0 or k < 0 or j < i ) return false;

    if ( k >= i and k <= j + 1 ) return false;

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
    return true;
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
  bool  e_reverse( UID i, UID j, double maxcapacity ) {
    assert (i < size() and j < size());

    if ( i<0 or j<0 or i >= path.size() or j >= path.size() )
      return false;

    int m = i;
    int n = j;

    if ( i == j ) return false;

    if ( i > j ) {
      m = j;
      n = i;
    }

    iterator itM = path.begin() + m;
    iterator itN = path.begin() + n;

    while ( itM < itN ) {  // TODO I dont think this comparison of iterators its correct
      std::iter_swap( itM, itN );
      itM++;
      itN--;
    }

    i < j ? evaluate(i, maxcapacity) : evaluate(j, maxcapacity);
    return true;
  }


  /*! \brief Evaluated: Remove a node from a path is e_erase.  */
  bool e_remove(UID i, double maxcapacity) {
    return e_erase(i,maxcapacity);
  }



  void evaluateOsrm( const std::string &osrmBaseUrl ) {
    assert( size() > 0 );
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
  }

  bool isOsrmTtimeValid() const {
    return path[path.size() - 1].isOsrmTtimeValid();
  }

  double getTotTravelTimeOsrm() const {
    return path[path.size() - 1].getTotTravelTimeOsrm();
  }
#endif  // 0


};

#endif  // SRC_BASECLASSES_TWPATH_H_


