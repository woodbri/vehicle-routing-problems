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
#ifndef BUCKET_H
#define BUCKET_H

#include <deque>
#include <vector>
#include <set>
#include <iostream>
#include <algorithm>
#include <limits>
#include <cassert>
#include "node.h"
//#include "twc.h"
//#include "plot.h"

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
 * - node id based tools
 * - position based tools
 * - other tools
*/
template <class knode> 
class TwBucket {

  protected:
    /*! \typedef typedef typename std::vector<std::vector<double> > TravelTimes
     * \brief Define an NxN vector array for holding travel times between nodes.
     */
    typedef typename std::vector<std::vector<double> > TravelTimes;

    /*! \fn double _MIN() const
     * \brief Define double -infinity
     */
    inline double _MIN() const { return (-std::numeric_limits<double>::max());};

    /*! \fn double _MAX() const
     * \brief Define double +infinity
     */
    inline double _MAX() const { return (std::numeric_limits<double>::max());};

    static  TravelTimes TravelTime; ///< Defines the travel time matrix
    std::deque<knode> path;         ///< Defines the bucket container


    /*! \class compNode
     * \brief A node comparision class for ordering nodes in a set.
     */
    class compNode{
       public:
       bool operator()(const knode &n1, const knode &n2) const {
         return (n1.getnid() < n2.getnid() );
       }
    };


    typedef unsigned long UID;
    typedef unsigned long POS;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;


  public:

    /*! \fn void setTravelTimes(const TravelTimes &_tt)
     * \brief Assign a travel time matrix to the \ref TwBucket
     */
    void setTravelTimes(const TravelTimes &_tt) {
        TravelTime=_tt;
    }

    /*! \fn double  timePCN(POS prev, POS curr, POS next) const
     * \brief Evaluates the time from the previous to current to next container.
     * \param[in] prev Position of the previous node in the path
     * \param[in] curr Position of the current node in the path
     * \param[in] next Position of the next node in the path
     *
     * simulates the following contiguous containers in the path
     * - prev curr next  
     *
     * \return travel time previous + service time current + travel time to next
     * \return infinity              if there is a TWV (timw window violation)
    */
    double  timePCN(POS prev, POS curr, POS next) const {
        assert ( prev < path.size() );
        assert ( curr < path.size() );
        assert ( next < path.size() );
        double result = path[curr].getArrivalTime()
                      + travelTime( path[prev], path[curr] );

        if ( result > path[curr].closes() ) return _MAX();
        if ( result < path[curr].opens() )
            result = path[curr].opens() - path[prev].getDepartureTime();

        return result + path[curr].getservicetime()
                      + travelTime(  path[curr], path[next] );
    }

    /*! \fn double  timePCN(POS &prev, POS &curr, const knode &dump) const
     * \brief Evaluates the time from the previous to current to the dump.
     * \param[in] prev Position of the previous node in the path
     * \param[in] curr Position of the current node in the path
     * \param[in] dump A reference to a dump node.
     *
     * simulates the following contiguous containers in the path
     * - prev curr dump
     *
     * \return travel time previous + service time current + travel time to dump
     * \return infinity              if there is a TWV (timw window violation)
    */
    double  timePCN(POS &prev, POS &curr, const knode &dump) const {
        assert ( prev < path.size() );
        assert ( curr < path.size() );
        double result = path[curr].getArrivalTime()
                      + travelTime( path[prev], path[curr] );

        if ( result  > path[curr].closes() ) return _MAX();

        return result + path[curr].getservicetime()
                      + travelTime( path[curr], dump );
    }

    /*! \fn double travelTime( const knode &from, const knode &to) const
     * \brief Fetch the travel time from Node to Node
     * \note Nodes do not need to be in the path.
    */
    double travelTime( const knode &from, const knode &to) const {
	    return travelTime( from.getnid(), to.getnid()  );
    }

    /*! \fn double travelTime(UID i, UID j) const
     * \brief Fetch the travel time from nodeId to nodeId
     * \note Nodes do not need to be in the path.
    */
    double travelTime(UID i, UID j) const {
        assert (i<TravelTime.size());
        assert (j<TravelTime.size());
        return TravelTime[i][j];
    }


    /*! \fn double getDeltaTime(const knode &node, const knode &dump) const
     * \brief Simulate changes of times within the path
     *
     * Simulates the following change of times within the path
     * - last dump
     * - last node dump
     *
     * and checks TWV and returns infinity if the occur at:
     * - node
     * - dump
     *
     * \return \f$ delta = tt_last,node + service(n) + tt_node,dump - tt_last,dump \f$
     * \return infinity when there is a TWV
    */
    double getDeltaTime(const knode &node, const knode &dump) const {
         knode last=path[path.size()-1];
         double nodeArrival = last.getDepartureTime() + travelTime(last,node);

         if (  node.latearrival(nodeArrival) ) return _MAX();
         if (  node.earlyarrival(nodeArrival) ) nodeArrival= node.opens();
         double dumpArrival =  nodeArrival + node.getservicetime() + travelTime(node,dump);
         if (  dump.latearrival(dumpArrival) ) return _MAX();
         if (  dump.earlyarrival(dumpArrival) ) dumpArrival= dump.opens();
         double delta = dumpArrival - last.getDepartureTime();
         return delta;
    }

    /*! \fn double getDeltaTimeAfterDump(const knode &dump, const knode &node ) const
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
    double getDeltaTimeAfterDump(const knode &dump, const knode &node ) const {
         double nodeArrival = dump.getDepartureTime() + travelTime(dump,node);

         if (  node.latearrival(nodeArrival) ) return _MAX();
         if (  node.earlyarrival(nodeArrival) ) nodeArrival= node.opens();

         double dumpArrival =  nodeArrival + node.getservicetime() + travelTime(node,dump);

         if (  dump.latearrival(dumpArrival) ) return _MAX();
         if (  dump.earlyarrival(dumpArrival) ) dumpArrival= dump.opens();

         double delta = dumpArrival + dump.getservicetime() - dump.getDepartureTime();
         return delta;
    }


    /*! \fn double getDeltaTimeSwap(UID pos1, UID pos2) const
     * \brief Compute the change in time related to swapping nodes in pos1 and pos2
     *
     * Simulate swapping nodes in pos1 and pos2 in the path and compute
     * the delta time impact that would have on the path.
     *
     * \param[in] pos1 Position of the node to be swapped.
     * \param[in] pos2 Position of the other node to be swapped.
     * \return The delta time or infinity if if creates a path violation.
    */
    double getDeltaTimeSwap(UID pos1, UID pos2) const {
        assert( pos1<path.size()-1 and pos2<path.size() );
#ifdef TESTED
    std::cout<<"Entering twBucket::getDeltaTimeSwap() \n";
#endif

        double delta, oldTime, newTime;
        // pos1 is the lowest
        if (pos1 > pos2) { int tmp=pos1; pos1=pos2; pos2=tmp;}

        //special case nidPrev nid1 nid2 nidNext
        if ( pos2 == pos1+1) {
            // nids invloved
            // in the same order the nodes are in the path
            int nidPrev, nid1, nid2, nidNext;
            nidPrev = path[pos1-1].getnid();
            nid1 = path[pos1].getnid();
            nid2 = path[pos2].getnid(); 
            if (pos2 != size()) nidNext = path[pos2+1].getnid();
            //                pos1-1  pos1  pos2  pos2+1
            // newpath looks: nidPrev nid2 nid1, nidNext

            // check for TWV
            if ( path[pos1-1].getDepartureTime() + TravelTime[nidPrev][nid2] > path[pos2].closes() ) return _MAX();
            if ( path[pos1-1].getDepartureTime() + TravelTime[nidPrev][nid2] + path[pos1].getservicetime() + TravelTime[nid2][nid1] > path[pos1].closes() ) return _MAX();

            // locally we are ok...  no capacity Violations
            // sum (services) remains constant
            if (pos2+1 == size()) {
                // newpath looks: nidPrev nid1 nid2,  DUMP in V
                //                pos1-1  pos1  pos2  pos2+1
                // newpath looks: nidPrev nid2 nid1,  DUMP in V
                // delta = new - old
                oldTime = path[pos2].getDepartureTime();
                newTime = path[pos1-1].getDepartureTime()
                        + TravelTime[nidPrev][nid2] + TravelTime[nid2][nid1];
                delta = oldTime - newTime;
            } else { 
                // oldpath looks: nidPrev nid1 nid2,  nidNext
                //                pos1-1  pos1  pos2  pos2+1
                // newpath looks: nidPrev nid2 nid1,  nidNext

                oldTime = path[pos2+1].getArrivalTime();
                newTime = path[pos1-1].getDepartureTime()
                        + TravelTime[nidPrev][nid2]
                        + TravelTime[nid2][nid1]
                        + TravelTime[nid1][nidNext] ;
                delta   =  oldTime - newTime;;
            }

            // check for TWV
            if (pos2+1 < size() and deltaGeneratesTV(delta,pos2+1)) return _MAX();
            return delta;
            // end of case when one node is after the other
        }

        // oldpath looks: nidPrev1 nid1 nidnext1    nidPrev2    nid2,  nidNext2
        //                pos1-1  pos1  pos1+1      pos2-1	    pos2    pos2+1
        // newpath looks: nidPrev1 nid2 nidnext1    nidPrev2,   nid1,  nidNext2
        double delta1 = getDeltaTime( path[pos2], pos1, pos1+1 );
        double delta2 = getDeltaTime( path[pos1], pos2, pos2+1 );

        // check if TWV is generated
        if ( (delta1 == _MAX() ) or  (delta2 == _MAX()) ) return _MAX();
        if ( deltaGeneratesTVupTo(delta1, pos1, pos2-1) ) return _MAX(); 
        if ( deltaGeneratesTV(delta1+delta2, pos2+1) ) return _MAX();

        // simple checks for cargo Violation
        if ( path[pos1].getdemand()== path[pos2].getdemand() and not path[size()-1].hascv()) return delta1+delta2;

        // check for cargo Violation Missing
        // if there is no dump  on the path: return  delta1 + delta2

        // if the share the same dump  return delta1 +delta2

        return delta1 + delta2;
    }


    /*! \fn double getDeltaTime(const knode &node, UID pos , UID pos1) const
     * \brief Compute the delta time of swapping node with the node at pos
     *
     * If the current path looks like prev -\> pos -\> pos1 then compute the
     * the change in time of swapping node for the node at pos, so the new
     * path would look like prev -\> node -\> pos1
     *
     * \param[in] node The node to evaluate if swapped with node at pos.
     * \param[in] pos The position of the node to be swapped.
     * \param[in] pos1 The next node following pos.
     * \return The change in cost or inifinty if a TWV would be generated.
     */
    double getDeltaTime(const knode &node, UID pos , UID pos1) const {
        assert(pos1<=path.size() );
        assert(pos>0 and pos1==(pos+1));

        if (pos==0 and path[pos].isdepot()) return _MAX();

        int nid=path[pos].getnid();
        int prev=path[pos-1].getnid();

        if ( path[pos-1].getDepartureTime() + TravelTime[prev][node.getnid()] > node.closes() ) return _MAX();
        if (pos1 ==size() ) return  TravelTime[prev][node.getnid()] + node.getservicetime() - (path[pos].getDepartureTime() - path[pos-1].getDepartureTime());

        int next = path[pos1].getnid();

        double delta  =  TravelTime[prev][node.getnid()]
                      + node.getservicetime()
                      + TravelTime[node.getnid()][next]  
                      - ( path[pos1].getArrivalTime()
                          - path[pos-1].getDepartureTime() ) ;
        return delta;
    }

    /*! \fn 
     * \brief 
     *
     *
     */
    double getDeltaTimeTVcheck(const knode &node, UID pos, UID pos1) const {
        assert(pos1<=path.size() );
        assert(pos>0 and pos1==(pos+1));

        double delta = getDeltaTime(node,pos,pos1);

        if ( path[pos-1].getDepartureTime() + TravelTime[ path[pos-1].getnid() ] [node.getnid() ] > node.closes() ) return _MAX();
        if (pos==size() ) return delta;
        if (deltaGeneratesTV(delta,pos1)) return _MAX();

        return delta;
    }




//to be used when inserting a node right before pos
double  getDeltaTime(const knode &node, UID pos) const {
     assert(pos<path.size() );
     if (pos==0 or path[pos].isdepot()) return _MAX();

     int nid=path[pos].getnid();
     int prev=path[pos-1].getnid();
     
     if (pos==size() ) return  TravelTime[prev][node.getnid()] + node.getservicetime();
     return TravelTime[prev][node.getnid()] + node.getservicetime() + TravelTime[node.getnid()][nid]  -   TravelTime[prev][nid];

}


double  getDeltaTimeTVcheck(const knode &node, UID pos) const {
     assert(pos<=path.size() );
     assert(pos>0);

     double delta = getDeltaTime(node,pos);
     
     if ( path[pos-1].getDepartureTime() + TravelTime[ path[pos-1].getnid() ][ node.getnid()]  > node.closes() ) return _MAX();
     if (pos==size() ) return delta;
     if (deltaGeneratesTV(delta,pos)) return _MAX();

     return delta;
}








bool deltaGeneratesTVupTo(double delta, UID pos, UID upto) const {
     assert(pos<path.size() and upto < size() and pos <= upto);
     bool flag = false;
     for (int i=pos;i<=upto;i++) // checking if the delta affects any node after it
        if ( path[i].getArrivalTime()+delta>path[i].closes() ) {flag=true;break;}
     return flag;
}

bool deltaGeneratesTV(double delta, UID pos) const {
     if (pos<size()) return  deltaGeneratesTVupTo(delta,pos,size()-1);
     else return false;
}


// other tools
    double segmentDistanceToPoint(UID i, const knode& n) const {
        assert(i+1<path.size());
        return n.distanceToSegment(path[i],path[i+1]);
    }

    void swap(UID i, UID j) { std::iter_swap(this->path.begin()+i,this->path.begin()+j);}
    bool swap(UID i, TwBucket<knode> &rhs,UID j) {
        assert ( i<size() and j<rhs.size() );
        std::iter_swap(path.begin()+i,rhs.path.begin()+j);
        return true;
    }


    void move(int fromi, int toj) {
        if (fromi == toj) return;
        if (fromi < toj){
            insert(this->path[fromi], toj + 1);
            erase(fromi);
        } else {
            insert(this->path[fromi], toj);
            erase(fromi + 1);
        }
    };


    void dumpid() const {dumpid("Twbucket");};
    void dumpid(const std::string &title) const {
        std::cout << title; 
        const_iterator it = path.begin();
        for (const_iterator it = path.begin(); it != path.end();it++)
            std::cout << " " << it->getid();
        std::cout << std::endl;
    };

    void dump() const {dump("Twbucket");};
    void dump(const std::string &title) const {
        std::cout << title; 
        const_iterator it = path.begin();
        for (const_iterator it = path.begin(); it != path.end();it++)
            std::cout << " " << it->getnid();
        std::cout << std::endl;
    };

//  set operations tools
    bool hasId(const knode &node) const { return hasid(node.getid()); };
    bool hasId(UID id) const {
        const_reverse_iterator rit = path.rbegin();
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getid()==id) return true;
              if(rit->getid()==id) return true;
        }
        return false;
    };


    bool has(const knode &node) const { return has(node.getnid()); };
    bool has(UID nid) const {
        const_reverse_iterator rit = path.rbegin();
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getnid()==nid) return true;
              if(rit->getnid()==nid) return true;
        }
        return false;
    };

    bool operator ==(const TwBucket<knode> &other) const  {
       if (size()!= other.size()) return false;
       if (size()==other.size()==0) return true;
       if ( ((*this)-other).size()!= 0) return false;
       if ( (other-(*this)).size()!= 0) return false;
       return true;
   }

    TwBucket<knode>& operator =(const TwBucket<knode> &other)  {
       TwBucket<knode> b=other;
       path.clear();
       path.insert(path.begin(),b.path.begin(),b.path.end());
       return *this;
    }

       

    // set doesnt mind order of nodes
    // UNION
    TwBucket<knode> operator +(const TwBucket<knode> &other) const  {
       std::set<knode,compNode> a;
       a.insert(path.begin(),path.end());
       a.insert(other.path.begin(),other.path.end());
       TwBucket<knode> b;
       b.path.insert(b.path.begin(),a.begin(),a.end());
       return b;
    }

    TwBucket<knode>& operator +=(const TwBucket<knode> &other)  {
       std::set<knode,compNode> a;
       a.insert(path.begin(),path.end());
       a.insert(other.path.begin(),other.path.end());
       path.clear();
       path.insert(path.begin(),a.begin(),a.end());
       return *this;
    }

    // INTERSECTION
    TwBucket<knode> operator *(const TwBucket<knode> &other) const  {
       std::set<knode,compNode> s1;
       std::set<knode,compNode> s2;
       std::set<knode,compNode> intersect;
       s1.insert(path.begin(),path.end());
       s2.insert(other.path.begin(),other.path.end());
       std::set_intersection( s1.begin(), s1.end(), s2.begin(), s2.end(),
              std::inserter( intersect, intersect.begin() ) );
       TwBucket<knode> b;
       b.path.insert(b.path.begin(),intersect.begin(),intersect.end());
       return b;
    }

    TwBucket<knode>& operator *=(const TwBucket<knode> &other)  {
       std::set<knode,compNode> s1;
       std::set<knode,compNode> s2;
       std::set<knode,compNode> intersect;
       s1.insert(path.begin(),path.end());
       s2.insert(other.path.begin(),other.path.end());
       std::set_intersection( s1.begin(), s1.end(), s2.begin(), s2.end(),
              std::inserter( intersect, intersect.begin() ) );
       path.clear();
       path.insert(path.begin(),intersect.begin(),intersect.end());
       return *this;
    }

    // DIFERENCE
    TwBucket<knode> operator -(const TwBucket<knode> &other) const  {
       std::set<knode,compNode> s1;
       std::set<knode,compNode> s2;
       std::set<knode,compNode> diff;
       s1.insert(path.begin(),path.end());
       s2.insert(other.path.begin(),other.path.end());
       std::set_difference( s1.begin(), s1.end(), s2.begin(), s2.end(),
             std::inserter( diff, diff.begin() ) );
       TwBucket<knode> b;
       b.path.insert(b.path.begin(),diff.begin(),diff.end());
       return b;
    }

    TwBucket<knode>& operator -=(const TwBucket<knode> &other)  {
       std::set<knode,compNode> s1;
       std::set<knode,compNode> s2;
       std::set<knode,compNode> diff;
       s1.insert(path.begin(),path.end());
       s2.insert(other.path.begin(),other.path.end());
       std::set_difference( s1.begin(), s1.end(), s2.begin(), s2.end(),
             std::inserter( diff, diff.begin() ) );
       path.clear();
       path.insert(path.begin(),diff.begin(),diff.end());
       return *this;
    }

//Last node on path data retreival
    double getTotTravelTime() const {
       assert (size());
       return path[size()-1].getTotTravelTime();
    };

    double getTotWaitTime() const {
       assert (size());
       return path[size()-1].getTotWaitTime();
    };

    double getTotServiceTime() const {
       assert (size());
       return path[size()-1].getTotServiceTime();
    };

    double getDumpVisits() const {
       assert (size());
       return path[size()-1].getDumpVisits();
    };

    double getDepartureTime() const {
       assert (size());
       return path[size()-1].getDepartureTime();
    };

    int getTwvTot() const {
       assert (size());
       return path[size()-1].gettwvTot();
    };

    int getCvTot() const {
       assert (size());
       return path[size()-1].gettwvTot();
    };

    double getTotCargo() const {
       assert (size());
       return path[size()-1].getcargo();
    };


//  ID based tools  to NID tools

    long int getNidFromId(UID id) const {
        const_reverse_iterator rit = path.rbegin(); 
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getid()==id) return it->getnid();
              if(rit->getid()==id) return rit->getnid();
        }
        return -1;
    };


    long int posFromId(UID id) const {
//        const_reverse_iterator rit = path.rbegin(); 
        for (const_iterator it = path.begin(); it!=path.end() ;it++/*,++rit*/) {
              if(it->getid()==id) return int(it-path.begin());
//              if(rit->getnid()==nid) return path.size()-int(path.rbegin()-rit)-1;
        }
        return -1;
    };


//  NID tools
    long int pos(const knode &node) const { return pos(node.getnid()); };
    long int pos(UID nid) const {
//        const_reverse_iterator rit = path.rbegin(); 
        for (const_iterator it = path.begin(); it!=path.end() ;it++/*,++rit*/) {
              if(it->getnid()==nid) return int(it-path.begin());
//              if(rit->getnid()==nid) return path.size()-int(path.rbegin()-rit)-1;
        }
        return -1;
    };

    std::deque<int> getpath() const {
        std::deque<int> p;
        for (const_iterator it = path.begin(); it != path.end();it++)
              p.push_back(it->getnid());
        return p;
    };
    

// deque like functions   POSITION based functions

    void insert(const knode &n, UID atPos) { 
        assert(atPos<=path.size());
        path.insert(path.begin() + atPos, n); 
    };
    void erase (int atPos) { 
        assert(atPos<path.size());
        path.erase(path.begin()+atPos); 
    };
    void erase (const knode &node) { 
        int atPos=pos(node.getnid());
        assert(atPos<path.size());
        path.erase(path.begin()+atPos); 
    };

    void erase (int fromPos, int toPos) { 
        assert(fromPos<path.size());
        assert(toPos<path.size());
         if (fromPos==toPos) path.erase(fromPos); //[fromPos]
         else if (fromPos<toPos) path.erase(path.begin()+fromPos,path.begin()+toPos); //[fromPos,toPos)
         else  path.erase(path.begin()+toPos,path.begin()+fromPos); //[toPos,fromPos)
    };

    void push_back(const knode& n) { path.push_back(n); };
    void push_front(const knode& n) { path.push_front(n); };
    void pop_back() { path.pop_back(); };
    void pop_front() { path.pop_front(); };
    void resize(unsigned int n) { 
         assert(n<=path.size());
         path.resize(n); };
    void clear() { path.clear(); };
    unsigned int max_size() const { return path.max_size(); };
    unsigned int size() const { return path.size(); };
    bool empty() const { return path.empty(); };
    std::deque<knode>& Path() { return path; }
    const std::deque<knode>& Path() const  { return path; }
    knode& operator[](unsigned int n) { 
         assert(n<path.size());
         return path[n]; };
    const knode&  operator[] (unsigned int n) const { 
         assert(n<path.size());
         return path[n]; };
    knode& at(int n) { 
         assert(n<path.size());
         return path.at(n); };
    const knode& at(int n) const  { 
         assert(n<path.size());
         return path.at(n); };
    knode& front() { return path.front(); };
    const knode& front() const { return path.front(); };
    knode& back() { return path.back(); };
    const knode& back() const { return path.back(); };


};

template <class knode>
std::vector<std::vector<double> > TwBucket<knode>::TravelTime ;


#endif


