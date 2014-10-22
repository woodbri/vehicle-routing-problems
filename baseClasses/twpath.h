#ifndef TWPATH_H
#define TWPATH_H

#include <deque>
#include <iostream>
#include <algorithm>
#include "node.h"
#include "twbucket.h"

/*
    Evaluation has to be done
*/

enum E_Ret {
    OK        = 0,
    NO_CHANGE = 1,
    INVALID   = 2
};


template <class knode> 
class Twpath : public TwBucket<knode> {
    // ------------------------------------------------------------------
    // TwBucket has the non evaluating methods
    // Twpath has the evaluating methods &
    //      inherits the all the non evaluating methods
    // ------------------------------------------------------------------

  private:
   
    typedef unsigned long UID;
    typedef typename std::deque<knode>::iterator iterator;
    typedef typename std::deque<knode>::reverse_iterator reverse_iterator;
    typedef typename std::deque<knode>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<knode>::const_iterator const_iterator;

  public:
    // ------------------------------------------------------------------
    // methods from TwBcuket class used in the evaluating functions
    // If a new evaluationg funtion uses a method of Twbucket not listed here
    //     add a new line with the correspondig name
    // ------------------------------------------------------------------
    using TwBucket<knode>::swap;
    using TwBucket<knode>::insert;
    using TwBucket<knode>::erase;
    using TwBucket<knode>::size;
    using TwBucket<knode>::path;


    /* operations within two  paths */
    E_Ret e_swap(UID i, double maxcap, Twpath<knode> &rhs,UID j, double rhs_maxcap) {
        assert ( i<size() and j<rhs.size() );
        if (i<0 or j<0 or i>size()-1 or j>rhs.size()-1) return INVALID;
        std::iter_swap(path.begin()+i,rhs.path.begin()+j);
        evaluate(i, maxcap);
        rhs.evaluate(j, rhs_maxcap);
        return OK;
    }



    // nodes handling within the same path 

    //  path with size 10:  (0......9) retsriction   0 <= i,j <= size-1
    // i.e. if the path has 10 nodes we can't move from
    // position 5000 OR can't move to position 5000  
    E_Ret e_move(UID fromi, UID toDest, double maxcapacity) {
        assert ( fromi<size() and toDest<size() );
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
        assert ( numb<=size() );
        if (numb<0 or numb>size()) return INVALID;
        path.resize(numb);
        //its reduced so the last node's value its not affected so no need of
        evalLast(maxcapacity); //<--- can this one be avoided????
        return OK;
    };

    E_Ret e_swap(UID i,UID j,double maxcapacity) {
        if (i==j) return NO_CHANGE;
        //if (i<0 or j<0 or i>size()-1 or j>size()-1) return INVALID;
        swap(i,j);
        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
        return OK;
    };



    // moves a range of nodes (i-j) to position k without reversing them
    // probably more efficient with iterators....
    // but this works
    E_Ret e_move(UID i, UID j, UID k, double maxcapacity) {
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



    // moves a range of nodes (i-j) to position k and reverses those nodes
    E_Ret e_movereverse(UID i, UID j, int k, double maxcapacity) {
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
    E_Ret e_reverse(UID i, UID j, double maxcapacity) {
        assert (i<size() and j<size());
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
            itM++;
            itN--;
        }

        i < j ? evaluate(i, maxcapacity): evaluate(j, maxcapacity);
        return OK;
    };

    E_Ret e_insert(const knode &n, UID at, double maxcapacity) {
        assert ( at <= size());
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

    E_Ret e_remove (UID i, double maxcapacity) {
        assert (i<size());
        if (i<0 or i>size()-1) return INVALID;
        path.erase(path.begin() + i);
        evaluate(i, maxcapacity);
        return OK;
    };

    /*****   EVALUATION   ****/
    void evaluate(UID from,double maxcapacity) {
        assert (from<=size()); //the equal just in case the last operation was erase

        if (from >= path.size()) from = size()-1;
        iterator it = path.begin()+from;

        while (it != path.end()){
            if (it==path.begin()) it->evaluate(maxcapacity);
            else it->evaluate(*(it-1),maxcapacity);
            it++;
        }

    };

    void evalLast(double maxcapacity) {
        assert (size()>0);
        evaluate(path.size()-1, maxcapacity);
    };

    void evaluate(double maxcapacity){
        assert (size()>0);
        evaluate(0,maxcapacity);
    };
    
    bool operator ==(const Twpath<knode> &other){
        if (size()!=other.size()) return false;
        iterator it = path.begin();
        iterator ito = other.path.begin();

        while (it != path.end()){
            if (it->getnid() != ito->getnid()) return false;
            ito++; it++;
        }
    }
         
    void dumpeval() const {
        for (int i=0;i<path.size();i++)
            path[i].dumpeval();
   }


    Twpath<knode>& operator =(const TwBucket<knode> &other){
         TwBucket<knode>::operator = (other);
         return *this;
    }

    Twpath<knode>& operator -=(const TwBucket<knode> &other){
       assert("Set operation not allowed on derived class of Twpath, Overload -= if this is required"=="");
    }
    Twpath<knode>& operator *=(const TwBucket<knode> &other){
       assert("Set operation not allowed on derived class of Twpath, Overload *= if this is required"=="");
    }
    Twpath<knode>& operator +=(const TwBucket<knode> &other){
       assert("Set operation not allowed on derived class of Twpath, Overload += if this is required"=="");
    }
////// (double underscore) (considers  Dumps;)

    bool createsViolation(UID from,double maxcapacity) {
#ifdef TESTED
std::cout<<"Entering twpath::createsViolation \n";
#endif
        assert (from<=size()); //the equal just in case the last operation was erase

        if (from >= path.size()) from = size()-1;

        iterator it = path.begin()+from;
        while (it != path.end()){
            if (it==path.begin()) it->evaluate(maxcapacity);
            else {
		it->evaluate(*(it-1),maxcapacity);
		if ( not it->feasable() ) return true;
            }
            if (it->isdump()) break;
            it++;
        }
        return false;
    };


    // doesnt insert if it creates a CV or TWV violation
    // dosnt move dumps
    bool e__insert(const knode &n, UID at, double maxcapacity ) {
#ifdef TESTED
std::cout<<"Entering twpath::e__insert \n";
#endif
        assert ( at <= size() );
        assert ( path[size()-1].feasable() );
        path.insert(path.begin() + at, n);
        if ( createsViolation(at, maxcapacity) ){
                erase(at);
		if (not createsViolation(at,maxcapacity) ) return false;
		assert (true==false);
        }
        assert ( path[size()-1].feasable() );
        return true;
    };

   bool e__adjustDumpsToMaxCapacity(int currentPos, const knode &dumpS,double maxcapacity) { //TODO convert to iterators
#ifdef TESTED
std::cout<<"Entering twpath::e__adjustDumpsToMaxCapacity \n";
#endif
	knode dumpSite=dumpS;
        int i=currentPos;
	while ( i<path.size()) {
	    if (path[i].isdump()) erase(i);
	    else i++;
        };
	evaluate(currentPos,maxcapacity); //make sure everything is evaluated
	if (path[size()-1].feasable()) return true; // no need to add a dump
        if (  path[size()-1].gettwvTot() ) return false; //without dumps its unfeasable

	//the path is dumplessi from the currentpos
	while ( path[size()-1].getcvTot() )  { //add dumps because of CV
	    //cycle until we find the first non CV
            for (i = path.size()-1; i>=currentPos-1 and path[i].getcvTot(); i--) {};
	    insert(dumpSite,i+1); // the dump should be after pos i
	    evaluate(i,maxcapacity);//reevaluate the rest of the route

#ifdef TESTED
std::cout<<"Entering twpath::e__adjustDumpsToMaxCapacity: inserted a dump \n";
dumpeval();
#endif 
	    //dont bother going to what we had before
	    if (  path[size()-1].feasable() ) return true; //added a dump and  is no cv and no twv 
	    if (  path[size()-1].gettwvTot() ) return false; // added a dump and created a twv, so why bother adding another dump
        };
	return  path[size()-1].feasable() ;
    };
	
};


#endif


