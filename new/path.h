#ifndef PATH_H
#define PATH_H

#include <deque>

#include "trashnode.h"

class Path {
  private:
    Trashnode home;
    std::deque<Trashnode> path;

  public:
    typedef std::deque<Trashnode>::iterator iterator;
    typedef std::deque<Trashnode>::const_iterator const_iterator;

    Trashnode& operator=(const Trashnode& n) { return path.operator=(n); };

    // element access
    Trashnode& operator[](size_type n) { return path[n]; };
    Trashnode& at(int n) { return path.at(n); };
    Trashnode& front() { return path.front(); };
    Trashnode& back() { return path.back(); };

    // iterators
    iterator begin() { return path.begin(); };
    iterator end() { return path.end(); };
    iterator rbegin() { return path.rbegin(); };
    iterator rend() { return path.rend(); };
    const_iterator cbegin() { return path.cbegin(); };
    const_iterator cend() { return path.cend(); };
    const_iterator crbegin() { return path.crbegin(); };
    const_iterator crend() { return path.crend(); };

    // Capacity
    size_type size() const { return path.size(); };
    size_type max_size() const { return path.max_size(); };
    void resize(size_type n) { path.resize(n); };
    bool empty() const { return path.empty(); };
    void shrink_to_fit() { path.shrink_to_fit(); };

    // modifiers
    void push_back(const Trashnode& n) { path.push_back(n); };
    void push_back(Trashnode&& n) { path.push_back(n); };
    void push_front(const Trashnode& n) { path.push_front(n); };
    void push_front(Trashnode&& n) { path.push_front(n); };
    void pop_back() { path.pop_back(); };
    void pop_front() { path.pop_front(); };
    iterator insert(const_iterator it, const Trashnode& n) { return path.insert(it, n); };
    iterator insert(const_iterator it, Trashnode&& n) { return path.insert(it, n); };
    iterator erase(const_iterator it) { return path.erase(it); };
    iterator erase(const_iterator first, const_iterator last) { return path.erase(first, last); };
    void clear() { path.clear(); };
    iterator emplace(const_iterator it, const Trashnode& n) { return path.emplace(it, n); };
    iterator emplace_front(const Trashnode& n) { return path.emplace_front(n); };
    iterator emplace_back(const Trashnode& n) { return path.emplace_back(n); };

};


#endif

