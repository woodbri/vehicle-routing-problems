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

    typedef unsigned int UID;
    typedef typename std::deque<knode> Path;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;

    // ------------------------------------------------------------------
    // These methods are NON-EVALUATING
    // and mirror those functions in std::deque
    // ------------------------------------------------------------------

  public:
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

    void insert(const knode &n, int atPos) { path.insert(path.begin() + atPos, n); };
    void erase (int atPos) { path.erase(path.begin()+atPos); };
    void erase (int fromPos, int toPos) { 
         if (fromPos==toPos) path.erase(fromPos);
         else if (fromPos<toPos) path.erase(path.begin()+fromPos,path.begin()+toPos); 
         else  path.erase(path.begin()+toPos,path.begin()+fromPos); 
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

    // how can we hande evaluation if we need the following???
    // clear() can be handled with:
    //      temp = path[0]; path.clear(); path.push_back(temp); ...


    // Capacity

    // ------------------------------------------------------------------
    // These methods are AUTO-EVALUATING
    // ------------------------------------------------------------------


    /* nodes handling within two  paths */
    bool e_swap(UID i, double maxcap, Twpath<knode> &rhs,UID j, double rhs_maxcap) {
        if (i>size()-1 or j>rhs.size()-1) return false;
        std::iter_swap(path.begin()+i,rhs.path.begin()+j);
        evaluate(i, maxcap);
        rhs.evaluate(j, rhs_maxcap);
        return true;
    }



    // nodes handling within the same path 
    //  path with size 10:  (0......9) retsriction   0 <= i,j <= size-1
    bool e_move(UID fromi, UID toDest, double maxcapacity) {
        if (fromi>size()-1 or toDest>size()-1) return false; //i.e. if the path has 10 nodes:  can't move from position 5000 OR can't move to position 5000  
        if (fromi == toDest) return true;  
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
        return true;
    };

    bool e_resize(UID numb,double maxcapacity) { 
        if (numb>size()) return false;
        path.resize(numb);
        //its reduced so the last node's value its not affected so no need of
        evalLast(maxcapacity); //????
        return true;
    };

    bool e_swap(UID i,UID j,double maxcapacity) {
        if (i==j) return true;
        if (i>size()-1 or j>size()-1) return false;
        swap(i,j);
        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
        return true;
    };

/*
    // moves a range of nodes (i-j) to position k without reversing them
    void e_move(int fromPos, int upToPos, int intoPos, double maxcapacity) {
        if (! (fromPos < upToPos and (intoPos > upToPos or intoPos < fromPos))) return;
        iterator itFromPos (path.begin()+fromPos);
        iterator itUpToPos (path.begin()+upToPos+1);
        iterator itIntoPos = path.begin()+intoPos;

        path.insert(itIntoPos, itFromPos, itUpToPos);
        if (intoPos > upToPos) { // moving range to right of the range
            path.erase(path.begin()+fromPos, path.begin()+upToPos);
            evaluate(fromPos,maxcapacity);
        } else {     // moving range to left of the range
            path.erase(path.begin()+fromPos+intoPos, path.begin()+upToPos+intoPos);
            evaluate(intoPos,maxcapacity);
        }
    };

*/


    // moves a range of nodes (i-j) to position k without reversing them
    bool e_move(int i, int j, int k, double maxcapacity) {
        if (! (i < j and (k > j or k < i))) return false;
        if (j>size()-1 or k>size()) return false;
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
        return true;
    }



    // moves a range of nodes (i-j) to position k and reverses those nodes
    bool e_movereverse(UID rangeFrom, UID rangeTo, UID destBeforePos, double maxcapacity) {
        if (! (rangeFrom < rangeTo and (destBeforePos > rangeTo or destBeforePos <= rangeFrom))) return false;
        if (rangeTo>size()-1 or destBeforePos>size()) return false; //avoiding wierd behaiviour
        
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
        return true;
    }

/*
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
*/

    // reverse the nodes from i to j in the path
    void e_reverse(int i, int j, double maxcapacity) {
        int m = i;
        int n = j;

        if (i == j) return;
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
    };

    void e_insert(const knode &n, int at, double maxcapacity) {
        path.insert(path.begin() + at, n);
        evaluate(at, maxcapacity);
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
/*
        for (int i=from; i<path.size(); i++) {
           if (i == 0) path[0].evaluate(maxcapacity);
           else path[i].evaluate(path[i-1], maxcapacity);
        };
*/
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
        for (int i=0; i<path.size(); i++){
              p.push_back(path[i].getnid());}
        return p;
    };

    void dump() const {
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


