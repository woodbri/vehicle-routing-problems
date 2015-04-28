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
#ifndef SRC_BASECLASSES_TWBUCKET_H_
#define SRC_BASECLASSES_TWBUCKET_H_

#include <deque>
#include <set>
#include <string>
#include <algorithm>


#ifdef DOVRPLOG
#include "./logger.h"
#endif

#include "./basictypes.h"
#include "./vrp_assert.h"
#include "./node.h"


/*! \class TwBucket
 * \brief A template class that provides deque like container with lots of additional functionality.
 *
 * TwBucket provides a set like container. It is used by \ref Twpath for
 * storage. It also provides several un-evaluated path operations. None of
 * the manipulation at this level is evaluated.
 *
 * The class provides:
 * - basic deque container like operations
 * - set operations
 * - node id based tools of the bucket/path
 * - position based tools of the bucket/path
 * - other tools
*/

template <class knode>
class TWC;

template <class knode>
class TwBucket {
 protected:
  typedef typename std::deque<knode>::iterator iterator;
  typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
  typedef typename
    std::deque<knode>::const_reverse_iterator const_reverse_iterator;
  typedef typename std::deque<knode>::const_iterator const_iterator;

  std::deque<knode> path;         ///< Defines the bucket container


  /*! \class compNode
    * \brief A node comparison class for ordering nodes in a set.
  */
  class compNode {
   public:
    bool operator()(const knode &n1, const knode &n2) const {
      return n1.nid() < n2.nid();
    }
  };



 private:
  /*!
    \param[in] prev:   prev node
   * \param[in] from:   from node
   * \param[in] middle: middle node
   * \param[in] to:     to node
   * \param[in] travelTimePrevFrom:   travel time from "prev" to "from" nodes
  */
  double  timePCN(const knode &prev, const knode &from, const knode &middle,
                  const knode &to, double travelTimePrevFrom) const {
    if ( from.isNotCompatibleIJ(prev)
         || middle.isNotCompatibleIJ(prev)
         || to.isNotCompatibleIJ(prev)
         || middle.isNotCompatibleIJ(from)
         || to.isNotCompatibleIJ(from)
         || to.isNotCompatibleIJ(middle))
       return VRP_MAX();
         

    double travelTimePrevFromMiddle = TravelTime(prev, from , middle);
    double travelTimeFromMiddle = travelTimePrevFromMiddle - travelTimePrevFrom;
    double travelTimePrevFromMiddleTo = TravelTime(prev, from , middle, to);
    double travelTimeMiddleTo  = travelTimePrevFromMiddleTo  - travelTimePrevFromMiddle;

    double arrive_from = prev.departureTime() + travelTimePrevFrom;

    if (from.lateArrival(arrive_from)) return VRP_MAX();
    if (from.earlyArrival(arrive_from)) arrive_from = from.opens();

    double depart_from = arrive_from + from.serviceTime();
    double arrive_middle = arrive_from + from.serviceTime() + travelTimeFromMiddle;

    if ( middle.lateArrival(arrive_middle) ) return VRP_MAX();
    if ( middle.earlyArrival(arrive_middle) ) arrive_middle = middle.opens();

    double arrive_to = arrive_middle + middle.serviceTime() + travelTimeMiddleTo;

    if (to.lateArrival(arrive_to)) return VRP_MAX();
    if (to.earlyArrival(arrive_to)) arrive_to = to.opens();

    return arrive_to - depart_from;
  }

  /*! @name timePCN = time previous-current-next 
     simulates the following order of nodes in the path: 
         prev from middle to
   *
   * \return  the time that takes to depart from "from" and arrive to "to" passing thru "middle"
   *
   * \return  ttfm + serv(m) + ttmt = arrivalTime(next) - departureTime(prev) when passing thru curr
   * \return infinity              if there is a TWV (time window violation)
   *
   * use when:
       prev from   is a section of the path and "prev" is the previous node of "from"
       from from   there is no previous container of from

  A private function does all the work
      the other ones are variations on the parameters
  */
  ///@{
 public:
  /*! \brief  the 3 nodes belong to the path */
  double  timePCN(POS from, POS middle, POS to) const {
    assert(from < path.size());
    assert(middle < path.size());
    assert(to < path.size());
    assert(middle != from);
    assert(middle != to);

    if ( from == 0 )
      return timePCN(path[from], path[from], path[middle], path[to], 0);
    else
      return timePCN(path[from - 1], path[from], path[middle], path[to],
                     path[from].travelTime());
  }

  /*! \brief  first 2 nodes belong to the path and the third doesnt */
  double timePCN(POS from, POS middle, const knode &dump) const {
    assert(from < path.size());
    assert(middle < path.size());
    assert(middle != from);

    if ( from == 0 )
      return timePCN(path[from], path[from], path[middle], dump, 0);
    else
      return timePCN(path[from - 1], path[from], path[middle], dump,
                     path[from].travelTime());
  }

  /*! \brief  \from node belong to the path and \middle and \dump dont */
  double  timePCN(POS from, const knode &middle, const knode &dump) const {
    assert(from < path.size());

    if ( from == 0 )
      return timePCN(path[from], path[from], middle, dump, 0);
    else
      return timePCN(path[from - 1], path[from], middle , dump,
                     path[from].travelTime());
  }

  /*! \brief  simulates a replacement of a node in the bucket

     previous path:
       from from+1 from+2
     simulated path:
       from middle from+2
  */
  double timePCN(POS from, const knode &middle) const {
    assert((from + 2) < path.size());

    if ( from == 0 )
      return timePCN(path[from], path[from], middle, path[from + 2], 0 );
    else
      return timePCN(path[from - 1], path[from], middle , path[from + 2],
                     path[from].travelTime());
  }
  /*! \brief  simulates an insertion of two nodes a node at the end of the bucket
         namely node and dump 

     previous path:
       last dump
     simulated path:
       last node dump
  */
  double timePCN(const knode &node, const knode &dump) const{
    knode last = path[path.size() - 1];
    return timePCN(path.size()-1, node, dump);
  }
  ///@}

 private:
  /*! @name TravelTime inline functions
    \brief useful inlines to fetch the travel time from Node to Node
   
   2 flavors:
      parameters are nodes
      parameters are node Nid (internal node id)
   No need to be within the bucket
  */
  ///@{
  inline double TravelTime(const knode &from, const knode &to) const {
    return TWC<knode>::Instance()->TravelTime( from.nid(), to.nid());
  }
  double TravelTime(const knode &from, const knode &middle,
                    const knode &to) const {
    return TWC<knode>::Instance()->TravelTime(from.nid(), middle.nid() , to.nid());
  }
  double TravelTime(const knode &prev, const knode &from, const knode &middle,
                    const knode &to) const {
    return TWC<knode>::Instance()->TravelTime(prev.nid(), from.nid(), middle.nid() , to.nid());
  }

  double TravelTime(UID i, UID j) const {
    return TWC<knode>::Instance()->TravelTime(i, j);
  }
  double TravelTime(UID i, UID j, UID k) const {
    return TWC<knode>::Instance()->TravelTime(i, j, k);
  }
  double TravelTime(UID i, UID j, UID k, UID l) const {
    return TWC<knode>::Instance()->TravelTime(i, j, k, l);
  }
  ///@}

#if 0
 public:
  /*! @name getDeltaTime
   * Simulate changes of times within the path
   \todo TODO check it returns a delta
  */

  /*!
   * \brief Simulate changes in travel times within the path
   *
   * Simulates the following change of travelTimes within the path
   * - dump
   * - dump node dump2
   *
   * and checks for TWV and returns infinity if the occur at:
   * - node
   * - dump2
   *
   * \return \f$ tt_dump,node + service(node) + tt_node,dump + service(dump) \f$
   * \return infinity when there is a TWV
  */
 // NOT USED
  double getDeltaTimeAfterDump(const knode &dump, const knode &node) const {
    double nodeArrival = dump.getDepartureTime() + TravelTime(dump, node);

    if ( node.lateArrival( nodeArrival) ) return VRP_MAX();

    if ( node.earlyArrival(nodeArrival) ) nodeArrival = node.opens();

    double dumpArrival =  nodeArrival + node.getServiceTime() +
                          TravelTime(node, dump);

    if ( dump.lateArrival(dumpArrival) ) return VRP_MAX();

    if ( dump.earlyArrival(dumpArrival) ) dumpArrival = dump.opens();

    double delta = dumpArrival + dump.getServiceTime() -
                   dump.getDepartureTime();
    return delta;
  }


  /*!
   * \brief Compute the change in time when swapping nodes in pos1 and pos2
   *
   * Simulate swapping nodes in pos1 and pos2 in the path and compute
   * the delta time impact that would have on the path.
   *
   * \param[in] pos1 Position of the node to be swapped.
   * \param[in] pos2 Position of the other node to be swapped.
   * \return The delta time or infinity if if creates a path violation.
  */
 // NOT USED
  double getDeltaTimeSwap(POS pos1, POS pos2) const {
    assert(pos1 < path.size() - 1 && pos2 < path.size());
#ifdef TESTED
    DLOG(INFO) << "Entering twBucket::getDeltaTimeSwap()";
#endif

    double delta, oldTime, newTime;

    // pos1 is the lowest
    if ( pos1 > pos2 ) { int tmp = pos1; pos1 = pos2; pos2 = tmp;}

    // special case nidPrev nid1 nid2 nidNext
    if ( pos2 == pos1 + 1 ) {
      // nids invloved
      // in the same order the nodes are in the path
      int nidPrev, nid1, nid2, nidNext;
      nidPrev = path[pos1 - 1].nid();
      nid1 = path[pos1].nid();
      nid2 = path[pos2].nid();

      if ( pos2 != size() ) nidNext = path[pos2 + 1].nid();

      //                pos1-1  pos1  pos2  pos2+1
      // newpath looks: nidPrev nid2 nid1, nidNext

      // check for TWV
      if ( path[pos1 - 1].getDepartureTime()
           + TravelTime[nidPrev][nid2] > path[pos2].closes() )
        return VRP_MAX();

      if ( path[pos1 - 1].getDepartureTime()
           + TravelTime[nidPrev][nid2] + path[pos1].getServiceTime()
           + TravelTime[nid2][nid1] > path[pos1].closes() )
        return VRP_MAX();

      // locally we are ok...  no capacity Violations
      // sum (services) remains constant
      if ( pos2 + 1 == size() ) {
        // newpath looks: nidPrev nid1 nid2,  DUMP in V
        //                pos1-1  pos1  pos2  pos2+1
        // newpath looks: nidPrev nid2 nid1,  DUMP in V
        // delta = new - old
        oldTime = path[pos2].getDepartureTime();
        newTime = path[pos1 - 1].getDepartureTime()
                  + TravelTime[nidPrev][nid2] + TravelTime[nid2][nid1];
        delta = oldTime - newTime;
      } else {
        // oldpath looks: nidPrev nid1 nid2,  nidNext
        //                pos1-1  pos1  pos2  pos2+1
        // newpath looks: nidPrev nid2 nid1,  nidNext

        oldTime = path[pos2 + 1].getArrivalTime();
        newTime = path[pos1 - 1].getDepartureTime()
                  + TravelTime[nidPrev][nid2]
                  + TravelTime[nid2][nid1]
                  + TravelTime[nid1][nidNext];
        delta   =  oldTime - newTime;;
      }

      // check for TWV
      if ( pos2 + 1 < size() && deltaGeneratesTV( delta, pos2 + 1 ) )
        return VRP_MAX();

      return delta;
      // end of case when one node is after the other
    }

    // oldpath looks: nidPrev1 nid1 nidnext1    nidPrev2    nid2,  nidNext2
    //                pos1-1  pos1  pos1+1      pos2-1      pos2    pos2+1
    // newpath looks: nidPrev1 nid2 nidnext1    nidPrev2,   nid1,  nidNext2
    double delta1 = getDeltaTime(path[pos2], pos1, pos1 + 1);
    double delta2 = getDeltaTime(path[pos1], pos2, pos2 + 1);

    // check if TWV is generated
    if ((delta1 == VRP_MAX()) || (delta2 == VRP_MAX())) return VRP_MAX();

    if ( deltaGeneratesTVupTo(delta1, pos1, pos2 - 1) ) return VRP_MAX();

    if ( deltaGeneratesTV(delta1 + delta2, pos2 + 1) ) return VRP_MAX();

    // simple checks for cargo Violation
    if ((path[pos1].getdemand() == path[pos2].getdemand())
        && !path[size() - 1].hascv())
      return delta1 + delta2;

    // check for cargo Violation Missing
    // if there is no dump  on the path: return  delta1 + delta2

    // if the share the same dump  return delta1 +delta2

    return delta1 + delta2;
  }


  /*!
   * \brief Compute the change in time when swapping node with the node at pos
   *
   * If the current path looks like prev -\> pos -\> pos1 then compute the
   * the change in time of swapping node for the node at pos, so the new
   * path would look like prev -\> node -\> pos1
   *
   * \param[in] node The node to evaluate if swapped with node at pos.
   * \param[in] pos The position of the node to be swapped.
   * \param[in] pos1 The next node following pos.
   * \return The change in cost or infinity if a TWV would be generated.
   */
 // NOT USED
  double getDeltaTime(const knode &node, POS pos , POS pos1) const {
    assert(pos1 <= path.size());
    assert(pos > 0 && pos1 == (pos + 1));

    if ( pos == 0 && path[pos].isdepot() ) return VRP_MAX();

    int nid = path[pos].nid();
    int prev = path[pos - 1].nid();

    if ( path[pos - 1].getDepartureTime()
         + TravelTime[prev][node.nid()] > node.closes() )
      return VRP_MAX();

    if ( pos1 == size() )
      return  TravelTime[prev][node.nid()]
              + node.getServiceTime()
              - (path[pos].getDepartureTime()
                 - path[pos - 1].getDepartureTime());

    int next = path[pos1].nid();

    double delta  =  TravelTime[prev][node.nid()]
                     + node.getServiceTime()
                     + TravelTime[node.nid()][next]
                     - (path[pos1].getArrivalTime()
                        - path[pos - 1].getDepartureTime());
    return delta;
  }

  /*!
   * \brief Compute the change in time when swapping node into pos in the path and do additional time violation checks.
   *
   * If the current path looks like prev -\> pos -\> pos1 then compute the
   * the change in time of swapping node for the node at pos, so the new
   * path would look like prev -\> node -\> pos1
   *
   * \param[in] node The node to evaluate if swapped with node at pos.
   * \param[in] pos The position of the node to be swapped.
   * \param[in] pos1 The next node following pos.
   * \return The change in cost or infinity if a TWV would be generated.
   */
  double getDeltaTimeTVcheck(const knode &node, POS pos, POS pos1) const {
    assert(pos1 <= path.size());
    assert(pos > 0 && pos1 == (pos + 1));

    double delta = getDeltaTime(node, pos, pos1);

    if ((path[pos - 1].getDepartureTime() + TravelTime[ path[pos - 1].nid() ] [node.nid() ])
         > node.closes()) 
      return VRP_MAX();

    if (pos == size()) return delta;

    if (deltaGeneratesTV( delta, pos1 )) return VRP_MAX();

    return delta;
  }


  /*!
   * \brief Compute the change in time of inserting node before pos in the path.
   *
   * Simulate inserting node before pos in the path and compute the resulting
   * change in time. No TW violations are checked.
   *
   * \param[in] node The node to be inserted in the simulation.
   * \param[in] pos The position before which the node will be inserted.
   * \return The change in travel time or infinity if the move is invalid.
   */
  double  getDeltaTime(const knode &node, POS pos) const {
    assert(pos < path.size());

    if ( pos == 0 || path[pos].isDepot() ) return VRP_MAX();

    int nid = path[pos].nid();
    int prev = path[pos - 1].nid();

    if ( pos == size() )
      return  TravelTime(prev, node.nid()) + node.getServiceTime();

    return TravelTime(prev, node.nid())
           + node.getServiceTime()
           + TravelTime(node.nid(), nid)
           - TravelTime(prev, nid);
  }


  /*!
   * \brief Compute the change in time of inserting node before pos in the path and check for TW violations..
   *
   * Simulate inserting node before pos in the path and compute the resulting
   * change in time and check for TW violations.
   *
   * \param[in] node The node to be inserted in the simulation.
   * \param[in] pos The position before which the node will be inserted.
   * \return The change in travel time or infinity if the move is invalid.
   */
 // NOT USED
  double  getDeltaTimeTVcheck(const knode &node, POS pos) const {
    assert(pos <= path.size());
    assert(pos > 0);

    double delta = getDeltaTime(node, pos);

    // check for TWV
    if ( path[pos - 1].getDepartureTime()
         + TravelTime[ path[pos - 1].nid() ][ node.nid()]
         > node.closes() ) return VRP_MAX();

    if ( pos == size() ) return delta;

    // check for TWV
    if ( deltaGeneratesTV( delta, pos ) ) return VRP_MAX();

    return delta;
  }


  /*!
   * \brief Check all nodes from pos to upto if adding delta would cause a violation.
   *
   * \param[in] delta The change in time to evaluate.
   * \param[in] pos The position to start evaluating.
   * \param[in] upto The position to stop evaluating.
   * \return true if delta would generate a time violation.
   */
  bool deltaGeneratesTVupTo(double delta, POS pos, POS upto) const {
    assert(pos < path.size() && upto < size() && pos <= upto);
    bool flag = false;

    // checking if the delta affects any node after it
    for ( int i = pos; i <= upto; i++ )
      if ( path[i].getArrivalTime() + delta > path[i].closes() ) {
        flag = true;
        break;
      }

    return flag;
  }

  /*!
   * \brief Check all nodes forward from pos if adding delta would cause a violation.
   *
   * \param[in] delta The change in time to evaluate.
   * \param[in] pos The position to start evaluating.
   * \return true if delta would generate a time violation.
   */
 // NOT USED
  bool deltaGeneratesTV(double delta, POS pos) const {
    if (pos < size())
      return  deltaGeneratesTVupTo(delta, pos, size() - 1);
    else
      return false;
  }
  ///@}
#endif // 0

 public:
  // ---------------- other tools ----------------------------------

  /*! \brief \returns the distance from a point to the segmnet (\b pos, \b pos+1)
   
    \warning assert(pos + 1 < path.size());
    \warning assert(path.size() > 1);

    \param[in] pos Position of start of segment.
    \param[in] node The node to compute the distance to.
    \return The shortest distance from node to line segment.
   */
  double segmentDistanceToPoint(POS pos, const knode &node) const {
    assert(path.size() > 1);
    assert(pos + 1 < path.size());
    return node.distanceToSegment(path[pos], path[pos + 1]);
  }


#ifdef DOVRPLOG
  /*! @name Dumping
   \brief Print the contents of the Twbucket
  */
  ///@{
  /*! \brief Using id as node identifiers with title "Twbucket". */
  void dumpid() const {dumpid("Twbucket");}


  /*! \brief Using id as node identifiers with user defined title.

   * \param[in] title Title to print with the output of the Twbucket.
   */
  void dumpid(const std::string &title) const {
    std::stringstream ss;
    ss << title;
    const_iterator it = path.begin();

    for ( const_iterator it = path.begin(); it != path.end(); it++ )
      ss << " " << it->id();

    DLOG(INFO) << ss.str();
  }

  /*! \brief Using nid as node identifiers with title "Twbucket".  */
  void dump() const {dump("Twbucket");}

  /*! \brief Using nid as node identifiers with title "Twbucket".  
   * \param[in] title Title to print with the output of the Twbucket.
   */
  void dump(const std::string &title) const {
    //std::stringstream ss;
    DLOG(INFO) << title;
    const_iterator it = path.begin();

    for (const_iterator it = path.begin(); it != path.end(); it++)
      it->dump();
    DLOG(INFO) << " <----- end \n";
    // DLOG(INFO) << ss.str();
  }
#endif
  ///@}


  /*! @name hasId

   \return true if a node with the same id is in the bucket.
  */
  ///@{
  /*! \brief \param[in] node uses the \b id of the node*/
  bool hasId(const knode &node) const {
     return hasid(node.id());
  }
  /*! \brief \param[in] id Uses the \b id */
  bool hasId(UID id) const {
    const_reverse_iterator rit = path.rbegin();

    for (const_iterator it = path.begin(); it != path.end() ; it++, ++rit) {
      if ( it->id() == id ) return true;
      if ( rit->id() == id ) return true;
    }

    return false;
  }
  ///@}


  /*! @name hasNId

   \return true if a node with the same nid was found in the bucket.
  */
  ///@{
  /*! \brief \param[in] node uses the \b nid of the node*/
  bool hasNid(const knode &node) const { 
    return hasNid(node.nid());
  }
  /*! \brief \param[in] id Uses the \b nid */
  bool hasNid(UID nid) const {
    const_reverse_iterator rit = path.rbegin();

    for (const_iterator it = path.begin(); it != path.end() ; it++, ++rit) {
      if ( it->nid() == nid ) return true;
      if ( rit->nid() == nid ) return true;
    }
    return false;
  }
  ///@}


  /*! @name Set operations based on the internal node id (nid) */
  ///@{
  /*!  * \brief True when \b this buckets is equal to the \b other bucket. */
  bool operator ==(const TwBucket<knode> &other) const  {
    if ( size() != other.size() ) return false;

    if ( size() == 0 && other.size() == 0 ) return true;

    if ( ((*this) - other).size() != 0 ) return false;

    if ( (other - (*this)).size() != 0 ) return false;

    return true;
  }


  /*! \brief Returns \b this  UNION \b other .  */
  TwBucket<knode> operator +(const TwBucket<knode> &other) const  {
    std::set<knode, compNode> a;
    a.insert(path.begin(), path.end());
    a.insert(other.path.begin(), other.path.end());
    TwBucket<knode> b;
    b.path.insert(b.path.begin(), a.begin(), a.end());
    return b;
  }

  /*! \brief Returns \b this INTERSECTION \b other .  */
  TwBucket<knode> operator *(const TwBucket<knode> &other) const  {
    std::set<knode, compNode> s1;
    std::set<knode, compNode> s2;
    std::set<knode, compNode> intersect;
    s1.insert(path.begin(), path.end());
    s2.insert(other.path.begin(), other.path.end());
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(),
                          std::inserter(intersect, intersect.begin()));
    TwBucket<knode> b;
    b.path.insert(b.path.begin(), intersect.begin(), intersect.end());
    return b;
  }

  /*! \brief Returns \b this DIFFERENCE \b other .  */
  TwBucket<knode> operator -(const TwBucket<knode> &other) const  {
    std::set<knode, compNode> s1;
    std::set<knode, compNode> s2;
    std::set<knode, compNode> diff;
    s1.insert(path.begin(), path.end());
    s2.insert(other.path.begin(), other.path.end());
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(),
                        std::inserter(diff, diff.begin()));
    TwBucket<knode> b;
    b.path.insert(b.path.begin(), diff.begin(), diff.end());
    return b;
  }
  ///@}


  /*! @name End of Path tools
    The end of the path is the \b last node of the path
  */
  ///@{
  const knode& last() const {
    assert(size());
    return  path[size() - 1];
  }

  /*! \brief \returns the total travel time of the path.  */
  double getTotTravelTime() const {
    assert(size());
    return last().totTravelTime();
  }

  /*! \brief \returns the duration of the path.  */
  double duration() const {
    assert(size());
    return last().duration();
  }
  
  /*! \brief \returns the total wait time of the path.  */
  double totWaitTime() const {
    assert(size());
    return last().totWaitTime();
  }

  /*! \brief \returns the total service time of the path */
  double totServiceTime() const {
    assert(size());
    return last().totServiceTime();
  }

  /*! \brief \returns the total number of dump visits of the path. */
  int dumpVisits() const {
    assert(size());
    return last().dumpVisits();
  }

  /*! \brief \returns the departure time of the last node in the path. */
  double departureTime() const {
    assert(size());
    return last().departureTime();
  }

  /*! \brief \returns the total number of time window violations in the path.  */
  int twvTot() const {
    assert(size());
    return last().twvTot();
  }

  /*! \brief \returns the total number of capacity violations in the path. */
  int cvTot() const {
    assert(size());
    return last().cvTot();
  }

  /*! \brief \returns the total cargo at the end of the path. */
  double cargo() const {
    assert(size());
    return last().cargo();
  }

  /*! \brief True when \b last node of path is feasable. */
  bool feasable() const {
    assert(size());
    return last().feasable();
  }

  /*! \brief True when \b last node of path is feasable. */
  bool feasable(double cargoLimit) const {
    assert(size());
    return last().feasable(cargoLimit);
  }

  /*! \brief True when \b last node of path has time window violation. */
  bool has_twv() const {
    assert(size());
    return last().has_twv();
  }

  /*! \brief True when \b last node of path has capacity violation. */
  bool has_cv(double cargoLimit) const {
    assert(size());
    return last().has_cv(cargoLimit);
  }
  ///@}

  // ---------- ID based tools  to NID tools ---------------------------

  /*!
   * \brief Get the internal node id associated with the user id.
   * \param[in] id The user id for the node.
   * \return The internal node id or -1 if user id was not found.
   * \todo TODO  put it in twc
   */
  UID getNidFromId(UID id) const {
    const_reverse_iterator rit = path.rbegin();

    for (const_iterator it = path.begin(); it != path.end() ; it++, ++rit) {
      if ( it->id() == id ) return it->nid();

      if ( rit->id() == id ) return rit->nid();
    }

    return 0;
  }


  /*!  * \brief Get the position in the path where id is located.

   * \param[in] id The user id for the node.
   * \return The position in the path or -1 if it is not found.
   */
  POS posFromId(UID id) const {
    for ( const_iterator it = path.begin(); it != path.end() ; it++ ) {
      if ( it->id() == id ) return POS(it - path.begin());
    }

    return 0;
  }


  /*! @name  position
     Gets the position of node in the bucket
  */
  ///@{
  /*!
   * \brief Get the position of node in the path
   * \param[in] node A node object that we want to locate in the path
   * \return returns the position of node in the path or 0 if it's not found.
   * \warning, if the position is 0, the user has to make sure it belongs to the bucket
   */
  POS pos(const knode &node) const { return position(node.nid()); }

  /*!
   * \brief Get the position of node id in the path
   * \param[in] nid The node id we want to locate in the path
   * \return The position of node id in the path or -1 if it's not found.
   */
  POS pos(UID nid) const {
    for ( const_iterator it = path.begin(); it != path.end() ; it++ ) {
      if ( it->nid() == nid ) return POS( it - path.begin() );
    }
    return 0;
  }
  ///@}


  /*! @name  mutators

     \warning No evaluation is done
  */
  ///@{
  /*! \brief  Both nodes are in the bucket

   * \param[in] i First node position to swap.
   * \param[in] j Second node position to swap.
   */
  void swap(POS i, POS j) {
    std::iter_swap(this->path.begin() + i, this->path.begin() + j);
  }

  /*! \brief  other node is in other bucket
   *
   * Swap nodes nodes between two buckets
   * - bucket1.swap( b1_pos, bucket2, b2_pos );
   *
   * The node in position b1_pos of bucket1 will be swapped with the node
   * in position b2_pos of bucket2.
   *
   * \param[in] b1_pos Position of node in bucket1
   * \param[in] bucket2 other bucket
   * \param[in] b2_pos Position of node in bucket2
   * \return true
   */
  bool swap(POS b1_pos, TwBucket<knode> &bucket2, POS b2_pos) {
    assert(b1_pos < size() && b2_pos < bucket2.size());
    std::iter_swap(path.begin() + b1_pos, bucket2.path.begin() + b2_pos);
    return true;
  }


  /*!  \brief Move node fromi to the new position of toj in this TwBucket */
  void move(int fromi, int toj) {
    if ( fromi == toj ) return;

    if ( fromi < toj ) {
      insert(this->path[fromi], toj + 1);
      erase(fromi);
    } else {
      insert(this->path[fromi], toj);
      erase(fromi + 1);
    }
  }
  ///@}


  /*! \brief Get a deque of nids that are in the path.

   * \return A deque of the nids in the path.
   */
  std::deque<int> getpath() const {
    std::deque<int> p;

    for ( const_iterator it = path.begin(); it != path.end(); it++ )
      p.push_back(it->nid());

    return p;
  }


  /*! @name   deque like functions
    assertions added
    Please refer to cpp deque documentation
    \returns True when the operation was completed
  */
  ///@{
  /*! \brief Insert node into deque
   * \param[in] atPos The position it should be inserted at
   * \param[in] node The node to insert
   */
  bool insert(const knode &node, POS atPos) {
    assert(atPos <= path.size());
    path.insert(path.begin() + atPos, node);
    return true;
  }


  /*! \brief Erase the node from deque at location atPos
   * \param[in] atPos The position of the node to be erased.
   */
  bool erase(POS atPos) {
    assert(atPos < path.size());
    path.erase(path.begin() + atPos);
    return true;
  }


  /* \brief Erase node from within the path.
   * \param[in] node The node to be erased.
   */
  bool erase(const knode &node) {
    if ( !hasNid(node) ) return false;
    int atPos = pos(node.nid());
    assert(atPos < path.size());
    path.erase(path.begin() + atPos);
    return true;
  }


  /*!  * \brief Erase all nodes between fromPos and toPos.

    \param[in] fromPos Position of the start of the range to be erased.
    \param[in] toPos Position of the last in the range to be erased.

    \warning Notice that the right side of the range is not included 
    when  ( fromPos < toPos )  range erased: [fromPos,toPos)
    when  ( fromPos > toPos )  range erased: [toPos,fromPos)

    \warning If fromPos and toPos are reversed it will still erase the range.
  */
  bool erase(POS fromPos, POS toPos) {
    assert(fromPos < path.size());
    assert(toPos < path.size());

    if ( fromPos == toPos ) {
      path.erase(fromPos);
    } else {
      if ( fromPos < toPos ) {  // [fromPos,toPos)
        path.erase(path.begin() + fromPos, path.begin() + toPos);
      } else {  // [toPos,fromPos)
        path.erase(path.begin() + toPos, path.begin() + fromPos);
      }
    }
  }

  bool push_back(const knode &node) {
    path.push_back(node);
    return true;
  }
  bool push_front(const knode &node) {
    path.push_front(node);
    return true;
  }
  iterator begin() { path.begin(); }
  iterator end() { path.begin(); }
  void pop_back() { path.pop_back(); }
  void pop_front() { path.pop_front(); }
  /*! \brief disables resizing to a larger bucket */
  void resize(UINT newSize) {
    assert(newSize <= path.size());
    path.resize(newSize);
  }
  void clear() { path.clear(); }
  unsigned int max_size() const { return path.max_size(); }
  unsigned int size() const { return path.size(); }
  bool empty() const { return path.empty(); }
  std::deque<knode>& Path() { return path; }
  const std::deque<knode>& Path() const  { return path; }
  knode& operator[](POS at) {
    assert(at < path.size());
    return path[at];
  }
  const knode& operator[] (POS at) const {
    assert(at < path.size());
    return path[at];
  }
  knode& at(POS pos) {
    assert(pos < path.size());
    return path.at(pos);
  }
  const knode& at(POS pos) const  {
    assert(pos < path.size());
    return path.at( pos );
  }
  knode& front() { return path.front(); }
  const knode& front() const { return path.front(); }
  knode &back() { return path.back(); }
  const knode& back() const { return path.back(); }
  ///@}
};


#endif  // SRC_BASECLASSES_TWBUCKET_H_


