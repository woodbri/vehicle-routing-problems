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

/*
Can be used as:

Set like container,
Twpath sections storage
   --allows several un-evaluated path operations
    
none of the manipulation at this level is evaluated
*/


template <class knode> 
class TwBucket {
/*  CLASS SUMMARY


--  other tools
--  set operations tools
--  NID base tools

-- deque like functions   POSITION based functions
   make available the deques container function to any derived class
   user can follow cpp deques container functions to see how they perform

*/

  protected:
    typedef typename std::vector<std::vector<double> > TravelTimes;
    inline double _MIN() const { return (-std::numeric_limits<double>::max());};
    inline double _MAX() const { return (std::numeric_limits<double>::max());};

    static  TravelTimes TravelTime;
    std::deque<knode> path;


class compNode{
   public:
   bool operator()(const knode &n1, const knode &n2) const {
     return (n1.getnid() < n2.getnid() );
   }
};


  private:
    typedef unsigned long UID;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;


  public:


   void setTravelTimes(const TravelTimes &_tt) {
      TravelTime=_tt;
   }

double  getDeltaTime(const knode &node, const knode &dump) const {
     int pos=path.size()-1;
     return TravelTime[pos][node.getnid()] + TravelTime[node.getnid()][dump.getnid()]  -   TravelTime[pos][dump.getnid()];
}

double  getDeltaTimeAfterDump(const knode &dump, const knode &lonelyNodeAfterDump ) const {
     if (  dump.getDepartureTime() + TravelTime[dump.getnid()][lonelyNodeAfterDump.getnid()]   > lonelyNodeAfterDump.closes() ) return _MAX();
     else  TravelTime[dump.getnid()][lonelyNodeAfterDump.getnid()] + TravelTime[lonelyNodeAfterDump.getnid()][dump.getnid()] + dump.getservicetime();
}


double  getDeltaTimeSwap(UID pos1, UID pos2) const {
     assert( pos1<path.size()-1 and pos2<path.size() );
#ifdef TESTED
std::cout<<"Entering twBucket::getDeltaTimeSwap() \n";
#endif

     double delta;
     if (pos1> pos2) { int tmp=pos1; pos1=pos2; pos2=tmp;} //pos1 is the lowest

     //special case nidPrev nid1 nid2 nidNext
     if ( pos2==pos1+1) {
	//nids invloved
	int nidPrev,nid1,nid2,nidNext; //in the same order the nodes are in the path
	nidPrev=path[pos1-1].getnid();
	nid1=path[pos1].getnid();
	nid2=path[pos2].getnid(); 
	if (pos2!=size()) nidNext=path[pos2+1].getnid();
        //newpath looks: nidPrev nid2 nid1, nidNext
	if ( path[pos1-1].getDepartureTime() + TravelTime[nidPrev][nid2] > path[pos2].closes() ) return _MAX();
	if ( path[pos1-1].getDepartureTime() + TravelTime[nidPrev][nid2] +  TravelTime[nid2][nid1] > path[pos1].closes() ) return _MAX();
        //locally we are ok...  no capacity Violations
        delta =   TravelTime[nidPrev][nid2] +  TravelTime[nid2][nid1]  
           		- ( TravelTime[nidPrev][nid1] +  TravelTime[nid1][nid2]) ;
        if (pos2!=size()-1) delta+=  TravelTime[nid2][nidNext] - TravelTime[nid1][nidNext] ;

        if (pos2+1 < size() and deltaGeneratesTV(delta,pos2+1)) return _MAX();
	return delta;
     }
     double delta1 = getDeltaTime( path[pos2] , pos1, pos1+1 );
     double delta2 = getDeltaTime( path[pos1] , pos2, pos2+1 );
     if ( (delta1 == _MAX() ) or  (delta2 == _MAX()) ) return _MAX();
     if (deltaGeneratesTVupTo (delta1, pos1, pos2-1) ) return _MAX();
     if (deltaGeneratesTV (delta1+delta2, pos2+1) ) return _MAX();

     //simple checks for cargo Violation
     if ( path[pos1].getdemand()== path[pos2].getdemand() and not path[size()-1].hascv()) return delta1+delta2;

     //check for cargo Violation Missing
     //if there is no dump  on the path: return  delta1 + delta2

     //if the share the same dump  return delta1 +delta2
     
     
     return delta1+delta2;
}


double  getDeltaTime(const knode &node, UID pos , UID pos1) const {
     assert(pos1<=path.size() );
     assert(pos>0 and pos1==(pos+1));

     if (pos==0 and path[pos].isdepot()) return _MAX();

     int nid=path[pos].getnid();
     int prev=path[pos-1].getnid();
     
     if ( path[pos-1].getDepartureTime() + TravelTime[prev][node.getnid()] > node.closes() ) return _MAX();
     if (pos1 ==size() ) return  TravelTime[prev][node.getnid()] + node.getservicetime() - (path[pos].getDepartureTime() - path[pos-1].getDepartureTime());

     int next = path[pos1].getnid();

     double delta  =  TravelTime[prev][node.getnid()] + node.getservicetime() + TravelTime[node.getnid()][next]  
		-  ( path[pos1].getArrivalTime() - path[pos-1].getDepartureTime() ) ;
     return delta;
}

double  getDeltaTimeTVcheck(const knode &node, UID pos, UID pos1) const {
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
     assert(pos<=path.size() );
     assert(pos>0);
     if (pos==0 and path[pos].isdepot()) return _MAX();

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
       return path[size()-1].getTotWaitTime();
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


