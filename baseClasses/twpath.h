#ifndef PATH_H
#define PATH_H

#include <deque>
#include <iostream>

#include "node.h"
#include "twnode.h"

/*
    TODO
    * would probably ne nice to have prev() and next()
    * this implies that we have whereami() that returns iterator
      pointing to self
    * what should prev() and next() do at the begin(0 and end() ?
*/

template <class knode> class Twpath {
  protected:
    knode home;
    knode dumpsite;
    std::deque<knode> path;

  public:
   /* node related */
   double getnid(int i) const  {return path[i].getnid();}; 
   double getx(int i) const  {return path[i].getx();}; 
   double gety(int i) const  {return path[i].gety();}; 
   bool isvalid(int i) const { return path[i].isvalid(); };
   bool issamepos(int i,int j) const { return path[i].isSamePos(path[j]); };
   bool issamepos(int i,int j,double tol) const { return path[i].isSamePos(path[j],tol); };
   bool issamepos(int i,const Node &n) const { return path[i].isSamePos(n); };
   bool issamepos(int i,const Node &n, double tol) const { return path[i].isSamePos(n,tol); };
   /* twnode related */
   bool checkintegrity(int i) const {return path[i].checkintegrity();};
   bool checkintegrity() const {  //of all nodes in path
        bool flag=true;
        for (int i=0; i<path.size();i++) {
            flag=flag and checkintegrity(i);
        }
        return flag;
   }
   bool hasdemand(int i) const { return path[i].hasdemand(); };
   bool hassupply(int i) const { return path[i].hassupply(); };
   bool hasnogoods(int i) const { return path[i].hasnogoods(); };
   bool earlyarrival(int i,const double D) const { return path[i].earlyarrival(D); };
   bool latearrival(int i,const double D) const { return path[i].latearrival(D); };


    int opens(int i) const {return path[i].opens();};
    int closes(int i) const {return path[i].closes();};
    int getdemand(int i) const{ return path[i].getdemand();};
    int getservicetime(int i) const{  return path[i].servicetime();};
    int windowlength(int i) const { return  path[i].windowlength(); };
    void dump(int i) const {path[i].dump();};

 
/* path specific operations */
    typedef typename std::deque<knode> nodepath;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;

    Twpath& operator=(const Twpath& n) {
        home = n.home;
        path = n.path;
        return *this;
    };

    knode& getdepot() { return home; };
    knode& getdumpsite() { return dumpsite; };

    void setdepot(knode& n) { home = n; };
    void setdumpsite(knode& n) { dumpsite = n; };

    // element access
    knode& operator[](unsigned int n) { return path[n]; };
    knode& at(int n) { return path.at(n); };
    knode& front() { return path.front(); };
    knode& back() { return path.back(); };

    // iterators
    iterator begin() { return path.begin(); };
    iterator end() { return path.end(); };
    iterator rbegin() { return path.rbegin(); };
    //iterator rend() { return path.rend(); };
    //const_iterator cbegin() { return path.cbegin(); };
    //const_iterator cend() { return path.cend(); };
    //const_iterator crbegin() { return path.crbegin(); };
    //const_iterator crend() { return path.crend(); };

    // Capacity
    unsigned int size() const { return path.size(); };
    unsigned int max_size() const { return path.max_size(); };
    void resize(unsigned int n) { path.resize(n); };
    bool empty() const { return path.empty(); };
    //void shrink_to_fit() { path.shrink_to_fit(); };

    // modifiers
    void push_back(const knode& n) { path.push_back(n); };
    void push_back(knode& n) { path.push_back(n); };
    void push_front(const knode& n) { path.push_front(n); };
    void push_front(knode& n) { path.push_front(n); };
    void pop_back() { path.pop_back(); };
    void pop_front() { path.pop_front(); };
    iterator insert(iterator it, const knode& n) { return path.insert(it, n); };
    //iterator insert(iterator it, Trashnode& n) { return path.insert(it, n); };
    //iterator erase(iterator it) { return path.erase(it); };
    //iterator erase(iterator first, iterator last) { return path.erase(first, last); };
    void clear() { path.clear(); };
    //iterator emplace(const_iterator it, const Trashnode& n) { return path.emplace(it, n); };
    //iterator emplace_front(const Trashnode& n) { return path.emplace_front(n); };
    //iterator emplace_back(const Trashnode& n) { return path.emplace_back(n); };

    void dump() {
        std::cout << "Path: " << home.getnid();
        for (int i=0; i<path.size(); i++)
            std::cout << ", " << path[i].getnid();
        std::cout << ", " << dumpsite.getnid()
                  << ", " << home.getnid()
                  << std::endl;
    };

};


#endif

