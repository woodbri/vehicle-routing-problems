#ifndef TWPATH_H
#define TWPATH_H

#include <deque>
#include <iostream>
#include <algorithm>
#include "node.h"

/*
    Evaluation has to be done
*/

enum E_Ret {
    OK        = 0,
    NO_CHANGE = 1,
    INVALID   = 2
};


template <class knode> class Twpath {
  protected:
    std::deque<knode> path;

    typedef unsigned long UID;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;


  public:
    // ------------------------------------------------------------------
    // These methods are NON-EVALUATING
    // ------------------------------------------------------------------
    inline void swap(UID i, UID j) { std::iter_swap(path.begin()+i,path.begin()+j);}

    void move(int fromi, int toj) {
        if (fromi == toj) return;
        if (fromi < toj){
            insert(path[fromi], toj + 1);
            erase(fromi);
        } else {
            insert(path[fromi], toj);
            erase(fromi + 1);
        }
    };

    double segmentDistanceToPoint(UID i, const knode& n, Node &point) const {
        return n.distanceToSegment(path[i],path[i+1],point);
    }

    // ------------------------------------------------------------------
    // These methods are NON-EVALUATING
    // and mirror those functions in std::deque
    // ------------------------------------------------------------------
    void insert(const knode &n, int atPos) { path.insert(path.begin() + atPos, n); };
    void erase (int atPos) { path.erase(path.begin()+atPos); };
    void erase (int fromPos, int toPos) { 
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


    /* nodes handling within two  paths */
    E_Ret e_swap(UID i, double maxcap, Twpath<knode> &rhs,UID j, double rhs_maxcap) {
        if (i<0 or j<0 or i>size()-1 or j>rhs.size()-1) return INVALID;
        std::iter_swap(path.begin()+i,rhs.path.begin()+j);
        evaluate(i, maxcap);
        rhs.evaluate(j, rhs_maxcap);
        return OK;
    }



    // nodes handling within the same path 
    //  path with size 10:  (0......9) retsriction   0 <= i,j <= size-1
    E_Ret e_move(UID fromi, UID toDest, double maxcapacity) {
        // i.e. if the path has 10 nodes we can't move from
        // position 5000 OR can't move to position 5000  
        if (fromi<0 or toDest<0 or fromi>size()-1 or toDest>size()-1)
            return INVALID;
        if (fromi == toDest) return NO_CHANGE;  
        if (fromi < toDest){
            if (toDest+1 > path.size())
                path.push_back(path[fromi]);  //I think this will never be executed
            else
                insert(path[fromi], toDest + 1);
            erase(fromi);
        } else {
            insert(path[fromi], toDest);
            erase(fromi + 1);
        }
        fromi < toDest ? evaluate(fromi, maxcapacity) : evaluate(toDest, maxcapacity);
        return OK;
    };

    E_Ret e_resize(UID numb,double maxcapacity) { 
        if (numb<0 or numb>size()) return INVALID;
        path.resize(numb);
        //its reduced so the last node's value its not affected so no need of
        evalLast(maxcapacity); //????
        return OK;
    };

    E_Ret e_swap(UID i,UID j,double maxcapacity) {
        if (i==j) return NO_CHANGE;
        if (i<0 or j<0 or i>size()-1 or j>size()-1) return INVALID;
        swap(i,j);
        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
        return OK;
    };



    // moves a range of nodes (i-j) to position k without reversing them
    E_Ret e_move(int i, int j, int k, double maxcapacity) {
        if (! (i <= j and (k > j or k < i))) return INVALID;
        if (j>size()-1 or k>size()) return INVALID;
        // moving range to right of the range
        if (k > j) {
            // if the length of the range is larger than the distance
            // being moved it is faster to move the intervening nodes in
            // the opposite direction
            if (j-i+1 > k-j-1) {
                return e_move(j+1, k-1, i, maxcapacity);
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
                return e_move(k, i-1, j+1, maxcapacity);
            }
            for (int n=i, m=0; n<=j; n++, m++) {
                knode temp = path[i+m];
                path.erase(path.begin()+i+m);
                path.insert(path.begin()+k+m, temp);
            }
        }
        //i < k ? path[i].evaluate(maxcapacity) : path[k].evaluate(maxcapacity);
        evaluate(maxcapacity);
        return OK;
    }


/*

    // moves a range of nodes (i-j) to position k and reverses those nodes
    E_Ret e_movereverse(UID rangeFrom, UID rangeTo, UID destBeforePos, double maxcapacity) {
        // path: 0 1 2 [3 4 5] 6 7 8 9
        // invalid moves are:
        //      rangeFrom > size-1 or rangeTo > size-1 or dest > size
        //      dest < 0 or to < 0 or from < 0 or to < from
        //      dest >= from and dest <= to+1
        if ( rangeFrom > path.size()-1 or rangeTo > path.size()-1 or
             destBeforePos > path.size() ) return INVALID;
        if ( rangeFrom < 0 or rangeTo < 0 or destBeforePos < 0
                or rangeTo < rangeFrom ) return INVALID;
        if ( destBeforePos >= rangeFrom and
             destBeforePos <= rangeTo+1 ) return INVALID;
        //if (! (rangeFrom < rangeTo and (destBeforePos > rangeTo or destBeforePos <= rangeFrom))) return INVALID;
        //if (rangeTo>size()-1 or destBeforePos>size()) return INVALID; //avoiding wierd behaiviour
        
        if (destBeforePos > rangeTo) { // moving range to right of the range
            reverse_iterator itFromPos (path.begin()+rangeTo+1);
            reverse_iterator itDownToPos (path.begin()+rangeFrom);
            iterator itIntoPos = path.begin()+destBeforePos;
            path.insert(itIntoPos, itFromPos, itDownToPos);
            path.erase(path.begin()+rangeFrom, path.begin()+rangeTo+1);
            evaluate(rangeFrom,maxcapacity);
        } else {     // moving range to left of the range
            iterator itn(path.begin()+rangeFrom);
            iterator itj(path.begin()+rangeTo);
            iterator itk(path.begin()+destBeforePos);
            for (int n=rangeFrom; n<=rangeTo; n++) {
                knode temp = *itn;
                path.erase(itn);
                path.insert(itk, temp);
                itk--;
            }
            evaluate(destBeforePos,maxcapacity);
        }
        return OK;
    }

*/


    // moves a range of nodes (i-j) to position k and reverses those nodes
    E_Ret e_movereverse(int i, int j, int k, double maxcapacity) {
        // path: 0 1 2 [3 4 5] 6 7 8 9
        // invalid moves are:
        //      rangeFrom > size-1 or rangeTo > size-1 or dest > size
        //      dest < 0 or to < 0 or from < 0 or to < from
        //      dest >= from and dest <= to+1
        if ( i > path.size()-1 or j > path.size()-1  or
             k > path.size() ) return INVALID;
        if ( i < 0 or j < 0 or k < 0 or j < i ) return INVALID;
        if ( k >= i and k <= j+1 ) return INVALID;

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
        evaluate(maxcapacity);
        return OK;
    }

    // reverse the nodes from i to j in the path
    E_Ret e_reverse(int i, int j, double maxcapacity) {
        if (i<0 or j<0 or i>=path.size() or j>=path.size()) return INVALID;
        int m = i;
        int n = j;

        if (i == j) return NO_CHANGE;
        if (i > j) {
            m = j;
            n = i;
        }
        iterator itM = path.begin()+m;
        iterator itN = path.begin()+n;

        while (itM < itN){
            std::iter_swap(itM,itN);
          //  knode tmp=*itM;
          //  *itM=*itN;
          //  *itN=tmp;
            itM++;
            itN--;
        }
/*
        while (m < n) {
            knode temp = path[m];
            path[m] = path[n];
            path[n] = temp;
            m++;
            n--;
        }
*/
        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
        return OK;
    };

    E_Ret e_insert(const knode &n, UID at, double maxcapacity) {
        if (at < 0 or at > size()) return INVALID;
        path.insert(path.begin() + at, n);
        evaluate(at, maxcapacity);
        return OK;
    };

    E_Ret e_push_back(const knode& n, double maxcapacity) {
        path.push_back(n);
        evalLast(maxcapacity);
        return OK;
    };
/*
    E_Ret e_push_back(knode& n, double maxcapacity) {
        path.push_back(n);
        evalLast(maxcapacity);
        return OK;
    };
*/
    E_Ret e_remove (int i, double maxcapacity) {
        if (i<0 or i>size()-1) return INVALID;
        path.erase(path.begin() + i);
        evaluate(i, maxcapacity);
        return OK;
    };

    /*****   EVALUATION   ****/
    void evaluate(int from,double maxcapacity) {

        if (from < 0 or from > path.size()) from = 0;
        iterator it = path.begin()+from;

        while (it != path.end()){
            if (it==path.begin()) it->evaluate(maxcapacity);
            else it->evaluate(*(it-1),maxcapacity);
            it++;
        }

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


    typedef Twpath<knode> Bucket;
};

//template <class knode>
//typedef Twpath<knode> Bucket;
//template <typename knode>
//class Bucket : public Twpath<knode> {};



#endif


