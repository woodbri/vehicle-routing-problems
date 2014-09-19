#ifndef BUCKET_H
#define BUCKET_H

#include <deque>
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

  protected:
    std::deque<knode> path;

  private:
    typedef unsigned long UID;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;


  public:
    double segmentDistanceToPoint(UID i, const knode& n, Node &point) const {
        assert(i<path.size());
        return n.distanceToSegment(path[i],path[i+1],point);
    }

// Functions for path section storage
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
    void resize(unsigned int n) { path.resize(n); };
    void clear() { path.clear(); };
    unsigned int max_size() const { return path.max_size(); };
    unsigned int size() const { return path.size(); };
    bool empty() const { return path.empty(); };
    std::deque<knode>& Path() { return path; }
    const std::deque<knode>& Path() const  { return path; }


    // how can we hande evaluation if we need the following???
    // clear() can be handled with:
    //      temp = path[0]; path.clear(); path.push_back(temp); ...


    // Capacity

    // ------------------------------------------------------------------
    // These methods are AUTO-EVALUATING
    // ------------------------------------------------------------------


    /*** ACCESSORS ***/

    std::deque<int> getpath() const {
        std::deque<int> p;
        for (const_iterator it = path.begin(); it != path.end();it++)
              p.push_back(it->getnid());
        return p;
    };

    UID pos(UID nid) const {
        const_reverse_iterator rit = path.rbegin(); 
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getnid()==nid) return int(it-path.begin());
              if(rit->getnid()==nid) return path.size()-int(path.rbegin()-rit)-1;
        }
        return -1;
    };

    bool in(UID nid) const {
        const_reverse_iterator rit = path.rbegin();
        for (const_iterator it = path.begin(); it!=path.end() ;it++,++rit) {
              if(it->getnid()==nid) return true;
              if(rit->getnid()==nid) return true;
        }
        return false;
    };

    

    void dump() const {
        std::cout << "Twpath: "; // << home.getnid();
        const_iterator it = path.begin();
        for (const_iterator it = path.begin(); it != path.end();it++)
            std::cout << " " << it->getnid();
        std::cout << std::endl;
    };

    // element access
    knode& operator[](unsigned int n) { return path[n]; };
    const knode&  operator[] (unsigned int n) const { return path[n]; };
    knode& at(int n) { return path.at(n); };
    const knode& at(int n) const  { return path.at(n); };
    knode& front() { return path.front(); };
    const knode& front() const { return path.front(); };
    knode& back() { return path.back(); };
    const knode& back() const { return path.back(); };


/*
    //  PATH specific operations


    // iterators
//    iterator begin() { return path.begin(); };
//    iterator end() { return path.end(); };
//    iterator rbegin() { return path.rbegin(); };
    //iterator rend() { return path.rend(); };
    //const_iterator cbegin() { return path.cbegin(); };
    //const_iterator cend() { return path.cend(); };
    //const_iterator crbegin() { return path.crbegin(); };
    //const_iterator crend() { return path.crend(); };


//iterator insert(iterator it, const knode& n) { return path.insert(it, n); };

    //void shrink_to_fit() { path.shrink_to_fit(); };

    // modifiers
    //iterator insert(iterator it, Trashnode& n) { return path.insert(it, n); };
    //iterator erase(iterator it) { return path.erase(it); };
    //iterator erase(iterator first, iterator last) { return path.erase(first, last); };
    //iterator emplace(const_iterator it, const Trashnode& n) { return path.emplace(it, n); };
    //iterator emplace_front(const Trashnode& n) { return path.emplace_front(n); };
    //iterator emplace_back(const Trashnode& n) { return path.emplace_back(n); };
*/


//    typedef Twpath<knode> Bucket;
};

//template <class knode>
//typedef Twpath<knode> Bucket;
//template <typename knode>
//class Bucket : public Twpath<knode> {};



#endif


