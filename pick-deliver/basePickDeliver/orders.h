#ifndef ORDERS_H
#define ORDERS_H

#include <deque>
#include <iostream>
#include <algorithm>
#include "bucketn.h"
#include "compatible.h"
#include "order.h"


class Orders {
  private:
    std::deque<Order> orders;
    std::deque< std::deque <bool> > compat;
    std::deque< std::deque <bool> > fifo;
    std::deque< std::deque <bool> > lifo;
    std::deque< std::deque <bool> > push;

    typedef typename std::deque<Order>::iterator iterator;
    typedef typename std::deque<Order>::reverse_iterator reverse_iterator;
    typedef typename std::deque<Order>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<Order>::const_iterator const_iterator;


  public:
    // Constructors
    Orders();   //to be used as a deque (aka. orders in vehicle)
    Orders(BucketN &nodes,const Dpnode &depot) ; //to be used as a deque with compatibility (aka. original orders)
    void makeOrders (BucketN &nodes,const Dpnode &depot);

    // compatibility
    bool isIncompatibleOrder(const Order &orderx, const Order &ordery) const;
    void setCompatibility( const Compatible &twc)  ;
    void dumpCompat() const;
    void dumpCompats(int kind) const;

    // setOperations
    void join(const Orders &other);

//    void erase (int fromPos, int toPos) ;
    void dump() const ;

    // mirror of deques functions
    void insert(const Order &o, int atPos) { orders.insert(orders.begin() + atPos, o); };
    void erase (int atPos) { orders.erase(orders.begin()+atPos); };
    void push_back(const Order& o) { orders.push_back(o); };
    void push_front(const Order& o) { orders.push_front(o); };
    void pop_back() { orders.pop_back(); };
    void pop_front() { orders.pop_front(); };
    void resize(unsigned int n) { orders.resize(n); };
    void clear() { orders.clear(); };
    unsigned int max_size() const { return orders.max_size(); };
    unsigned int size() const { return orders.size(); };
    bool empty() const { return orders.empty(); };
    
    // access
    Order& operator[](unsigned int n) { return orders[n]; };
    const Order&  operator[] (unsigned int n) const { return orders[n]; };
    Order& at(int n) { return orders.at(n); };
    const Order& at(int n) const  { return orders.at(n); };
    Order& front() { return orders.front(); };
    const Order& front() const { return orders.front(); };
    Order& back() { return orders.back(); };
    const Order& back() const { return orders.back(); };
    iterator begin() {return orders.begin();};
    iterator end() {return orders.end();};

};


#endif


