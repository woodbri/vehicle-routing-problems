#ifndef TWPATH_H
#define TWPATH_H

#include <deque>
#include <iostream>
#include <algorithm>

/*
    Evaluation has to be done

*/

template <class knode> class Twpath {
  protected:
    std::deque<knode> path;


    // ------------------------------------------------------------------
    // These methods are NON-EVALUATING
    // and mirror those functions in std::deque
    // ------------------------------------------------------------------

  public:
    void swap(int i, int j) {
        knode temp = path[i];
        path[i] = path[j];
        path[j] = temp;
    } 

    void move(int fromi, int toj) {
        if (fromi == toj) return;
        if (fromi < toj){
            insert(path[fromi], toj + 1);
            remove(fromi);
        } else {
            insert(path[fromi], toj);
            remove(fromi + 1);
        }
    };

    void insert(const knode &n, int at) { path.insert(path.begin() + at, n); };
    void remove (int i) { path.erase(path.begin()+i); };
    void push_back(const knode& n) { path.push_back(n); };
    void push_back(knode& n) { path.push_back(n); };
    void push_front(const knode& n) { path.push_front(n); };
    void push_front(knode& n) { path.push_front(n); };
    void pop_back() { path.pop_back(); };
    void pop_front() { path.pop_front(); };

    // how can we hande evaluation if we need the following???
    // clear() can be handled with:
    //      temp = path[0]; path.clear(); path.push_back(temp); ...

    void resize(unsigned int n) { path.resize(n); };
    void clear() { path.clear(); };

    // Capacity
    unsigned int max_size() const { return path.max_size(); };
    unsigned int size() const { return path.size(); };
    bool empty() const { return path.empty(); };

    // ------------------------------------------------------------------
    // These methods are AUTO-EVALUATING
    // ------------------------------------------------------------------



    /* nodes handling within two  paths */
    void e_swap(int i, double maxcap, Twpath<knode> &rhs, int j, double rhs_maxcap) {
        knode temp = path[i];
        path[i] = rhs.path[j];
        rhs.path[j] = temp;
        evaluate(i, maxcap);
        rhs.evaluate(j, rhs_maxcap);
    }



    /* nodes handling within the same path */
    void e_move(int fromi, int toj, double maxcapacity) {
        if (fromi == toj) return;
        if (fromi < toj){
            if (toj+1 > path.size())
                path.push_back(path[fromi]);
            else
                insert(path[fromi], toj + 1);
            e_remove(fromi, maxcapacity);
        } else {
            int sz = path.size();
            insert(path[fromi], toj);
            e_remove(std::min(sz, fromi + 1), maxcapacity);
        }
    };

    void e_resize(unsigned int at,double maxcapacity) { 
        path.resize(at);
        //theoricamente funciona sin mas
    };

    void e_swap(int i,int j,double maxcapacity) {
        if (i==j) return;
        knode temp = path[i];
        path[i] = path[j];
        path[j] = temp;
        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
    };


    // moves a range of nodes (i-j) to position k without reversing them
    void e_move(int i, int j, int k, double maxcapacity) {
        if (! (i < j and (k > j or k < i))) return;
        // moving range to right of the range
        if (k > j) {
            // if the length of the range is larger than the distance
            // being moved it is faster to move the intervening nodes in
            // the opposite direction
            if (j-i+1 > k-j-1) {
                e_move(j+1, k-1, i, maxcapacity);
                return;
            }
            for (int n=i, m=0; n<=j; n++, m++) {
                knode temp = path[i];
                path.erase(path.begin()+i);
                path.insert(path.begin()+k-1, temp);
            }
        }
        // moving range to left of the range
        else {
            // if the length of the range is larger than the distance
            // being moved it is faster to move the intervening nodes in
            // the opposite direction
            if (j-i+1 > i-k) {
                e_move(k, i-1, j+1, maxcapacity);
                return;
            }
            for (int n=i, m=0; n<=j; n++, m++) {
                knode temp = path[i+m];
                path.erase(path.begin()+i+m);
                path.insert(path.begin()+k+m, temp);
            }
        }
        //i < k ? path[i].evaluate(maxcapacity) : path[k].evaluate(maxcapacity);
        evaluate(maxcapacity);
    }


    // moves a range of nodes (i-j) to position k and reverses those nodes
    void e_movereverse(int i, int j, int k, double maxcapacity) {
        if (! (i < j and (k > j or k < i))) return;
        // moving range to right of the range
        if (k > j) {
            for (int n=i, m=1; n<=j; n++, m++) {
                knode temp = path[i];
                path.erase(path.begin()+i);
                path.insert(path.begin()+k-m, temp);
            }
        }
        // moving range to left of the range
        else {
            for (int n=i; n<=j; n++) {
                knode temp = path[n];
                path.erase(path.begin()+n);
                path.insert(path.begin()+k, temp);
            }
        }
        //i < k ? path[i].evaluate(maxcapacity) : path[k].evaluate(maxcapacity);
        evaluate(maxcapacity);
    }


    // reverse the nodes from i to j in the path
    void e_reverse(int i, int j, double maxcapacity) {
        int m = i;
        int n = j;

        if (i == j) return;
        if (i > j) {
            m = j;
            n = i;
        }
        while (m < n) {
            knode temp = path[m];
            path[m] = path[n];
            path[n] = temp;
            m++;
            n--;
        }
        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
    };

    void e_insert(const knode &n, int at, double maxcapacity) {
        path.insert(path.begin() + at, n);
        path[at].evaluate(maxcapacity);
    };

    void e_push_back(const knode& n, double maxcapacity) {
        path.push_back(n);
        evalLast(maxcapacity);
    };

    void e_push_back(knode& n, double maxcapacity) {
        path.push_back(n);
        evalLast(maxcapacity);
    };

    void e_remove (int i, double maxcapacity) {
        path.erase(path.begin() + i);
        evaluate(i, maxcapacity);
    };

    /*****   EVALUATION   ****/
    void evaluate(int from,double maxcapacity) {

        if (from < 0 or from > path.size()) from = 0;
        for (int i=from; i<path.size(); i++) {
            if (i == 0) path[0].evaluate(maxcapacity);
            else path[i].evaluate(path[i-1], maxcapacity);
        };
    };

    void evalLast(double maxcapacity) {
        evaluate(path.size()-1, maxcapacity);
    };

    void evaluate(double maxcapacity){
        evaluate(0,maxcapacity);
    };
    
    /*** ACCESSORS ***/

    std::deque<int> getpath() const {
        std::deque<int> p;
        for (int i=0; i<path.size(); i++){
              p.push_back(path[i].getnid());}
        return p;
    };

    void dump() {
        std::cout << "Twpath: "; // << home.getnid();
        for (int i=0; i<path.size(); i++)
            std::cout << ", " << path[i].getnid();
        std::cout << std::endl;
    };

    // element access
    knode& operator[](unsigned int n) { return path[n]; };
    knode  operator[] (unsigned int n) const { return path[n]; };
    knode& at(int n) { return path.at(n); };
    knode at(int n) const  { return path.at(n); };
    knode& front() { return path.front(); };
    knode front() const { return path.front(); };
    knode& back() { return path.back(); };
    knode back() const { return path.back(); };


/*
    //  PATH specific operations

    typedef typename std::deque<knode> nodepath;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;



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


};


#endif

