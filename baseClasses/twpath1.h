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

template <class Knode> class Twpath {
  protected:
    Knode home;
    Knode dumpsite;
    std::deque<Knode> path;

  public:
   /* node related */
   double getnid(int i) const  {return path[i].getnid();}; 
   double getx(int i) const  {return path[i].getx();}; 
   double gety(int i) const  {return path[i].gety();}; 
   double distance(int i, int j) {return path[i].distance(path[j]);}
   double distance(int i, Knode n) {return path[i].distance(n);}
   bool isvalid(int i) const { return path[i].isvalid(); };
   bool issamepos(int i,int j) const { return path[i].isSamePos(path[j]); };
   bool issamepos(int i,int j,double tol) const { return path[i].isSamePos(path[j],tol); };
   bool issamepos(int i,const Knode &n) const { return path[i].isSamePos(n); };
   bool issamepos(int i,const Knode &n, double tol) const { return path[i].isSamePos(n,tol); };
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
   bool ontime(int i, const double D) const {return not earlyarrival(D) and not latearrival(D);};


    int opens(int i) const {return path[i].opens();};
    int closes(int i) const {return path[i].closes();};
    int getdemand(int i) const{ return path[i].getdemand();};
    int getservicetime(int i) const{  return path[i].servicetime();};
    int windowlength(int i) const { return  path[i].windowlength(); };
    void dump(int i) const {path[i].dump();};

 
/* path specific operations */
    typedef typename std::deque<Knode> nodepath;
    typedef typename std::deque<Knode>::iterator iterator;
    typedef typename std::deque<Knode>::const_iterator const_iterator;

    Twpath& operator=(const Twpath& n) {
        home = n.home;
        path = n.path;
        return *this;
    };

    Knode& getdepot() { return home; };
    Knode& getdumpsite() { return dumpsite; };

    void setdepot(Knode& n) { home = n; };
    void setdumpsite(Knode& n) { dumpsite = n; };

    // element access
    Knode& operator[](unsigned int n) { return path[n]; };
    Knode& at(int n) { return path.at(n); };
    Knode& front() { return path.front(); };
    Knode& back() { return path.back(); };

    // iterators
    iterator begin() { return path.begin(); };
    iterator end() { return path.end(); };
    iterator rbegin() { return path.rbegin(); };
    iterator rend() { return path.rend(); };
    const_iterator cbegin() { return path.cbegin(); };
    const_iterator cend() { return path.cend(); };
    const_iterator crbegin() { return path.crbegin(); };
    const_iterator crend() { return path.crend(); };


    //operators
//     Knode& operator[] (const int nIndex) {return path[nIndex];};

    // Capacity
    unsigned int size() const { return path.size(); };
    unsigned int max_size() const { return path.max_size(); };
    void resize(unsigned int n) { path.resize(n); };
    bool empty() const { return path.empty(); };
    //void shrink_to_fit() { path.shrink_to_fit(); };

    // modifiers
    void push_back(const Knode& n) { path.push_back(n); };
    void push_back(Knode& n) { path.push_back(n); };
    void push_front(const Knode& n) { path.push_front(n); };
    void push_front(Knode& n) { path.push_front(n); };
    void pop_back() { path.pop_back(); };
    void pop_front() { path.pop_front(); };
    void insert (int at, const Knode &n) { path.insert(path.begin()+at,n);};
    void insert (int at, Knode &n) { path.insert(path.begin()+at,n);};
    void erase (int at) { path.erase(path.begin()+at);};

    //iterator insert(iterator it, const Knode& n) { return path.insert(it, n); };
    //iterator insert(iterator it, Knode& n) { return path.insert(it, n); };
    iterator erase(iterator it) { return path.erase(it); };
    iterator erase(iterator first, iterator last) { return path.erase(first, last); };
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

