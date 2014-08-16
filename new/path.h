#ifndef PATH_H
#define PATH_H

#include <deque>
#include <iostream>

#include "trashnode.h"

/*
    TODO
    * would probably ne nice to have prev() and next()
    * this implies that we have whereami() that returns iterator
      pointing to self
    * what should prev() and next() do at the begin(0 and end() ?
*/

class Path {
  protected:
    Trashnode home;
    Trashnode dumpsite;
    std::deque<Trashnode> path;

  public:
    typedef std::deque<Trashnode> Trashnodepath;
    typedef Trashnodepath::iterator iterator;
    typedef Trashnodepath::const_iterator const_iterator;

    Path& operator=(const Path& n) {
        home = n.home;
        path = n.path;
        return *this;
    };

    Trashnode getdepot() const { return home; };
    Trashnode getdumpsite() const { return dumpsite; };

    void setdepot(Trashnode& n) { home = n; };
    void setdumpsite(Trashnode& n) { dumpsite = n; };

    // element access
    Trashnode& operator[](unsigned int n) { return path[n]; };
    Trashnode& at(int n) { return path.at(n); };
    Trashnode& front() { return path.front(); };
    Trashnode& back() { return path.back(); };

    // iterators
    iterator begin() { return path.begin(); };
    iterator end() { return path.end(); };
    //iterator rbegin() { return path.rbegin(); };
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
    void push_back(const Trashnode& n) { path.push_back(n); };
    void push_back(Trashnode& n) { path.push_back(n); };
    void push_front(const Trashnode& n) { path.push_front(n); };
    void push_front(Trashnode& n) { path.push_front(n); };
    void pop_back() { path.pop_back(); };
    void pop_front() { path.pop_front(); };
    iterator insert(iterator it, const Trashnode& n) { return path.insert(it, n); };
    iterator insert(iterator it, Trashnode& n) { return path.insert(it, n); };
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

