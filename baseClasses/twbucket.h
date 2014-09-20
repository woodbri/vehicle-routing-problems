#ifndef BUCKET_H
#define BUCKET_H

#include <deque>
#include <set>
#include <iostream>
#include <algorithm>
#include <cassert>
#include "node.h"

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
// other tools
    double segmentDistanceToPoint(UID i, const knode& n, Node &point) const {
        assert(i<path.size());
        return n.distanceToSegment(path[i],path[i+1],point);
    }

    void swap(UID i, UID j) { std::iter_swap(this->path.begin()+i,this->path.begin()+j);}

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

    void dump() const {
        std::cout << "Twpath: "; // << home.getnid();
        const_iterator it = path.begin();
        for (const_iterator it = path.begin(); it != path.end();it++)
            std::cout << " " << it->getnid();
        std::cout << std::endl;
    };

//  set operations tools
    bool in(UID nid) const {
        const_reverse_iterator rit = path.rbegin();
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getnid()==nid) return true;
              if(rit->getnid()==nid) return true;
        }
        return false;
    };

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



//  NID based tools

    UID pos(UID nid) const {
        const_reverse_iterator rit = path.rbegin(); 
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getnid()==nid) return int(it-path.begin());
              if(rit->getnid()==nid) return path.size()-int(path.rbegin()-rit)-1;
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



#endif


