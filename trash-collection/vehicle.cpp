

#include <iostream>
#include <deque>

#include "twpath.h"
#include "vehicle.h"


void Vehicle::dump() const {
    std::cout << "---------- Vehicle ---------------" << std::endl;
    std::cout << "maxcapacity: " << getmaxcapacity() << std::endl;
    std::cout << "cargo: " << getcargo() << std::endl;
    std::cout << "duration: " << getduration() << std::endl;
    std::cout << "cost: " << getcost() << std::endl;
    std::cout << "TWV: " << getTWV() << std::endl;
    std::cout << "CV: " << getCV() << std::endl;
    std::cout << "w1: " << getw1() << std::endl;
    std::cout << "w2: " << getw2() << std::endl;
    std::cout << "w3: " << getw3() << std::endl;
    std::cout << "path nodes: -----------------" << std::endl;
    path.dump();
/*
    std::cout << "--------- dumpeval ----------" << std::endl;
    for (int i=0;i<path.size();i++){
        std::cout<<"\npath stop #:"<<i<<"\n";
        path[i].dumpeval();
    }
    std::cout<<"\ndumpsite:"<<"\n";
    dumpsite.dumpeval();
    std::cout<<"\nBack to depot:"<<"\n";
    backToDepot.dumpeval();
    std::cout <<"TOTAL COST="<<cost <<"\n";
*/
}


void Vehicle::dumppath() const {
    path.dump();
}


std::deque<int> Vehicle::getpath() const {
      std::deque<int> p;
      p = path.getpath();
      p.push_front(getdepot().getnid());
      p.push_back(getdumpsite().getnid());
      p.push_back(getdepot().getnid());
      return p;
}


void Vehicle::push_back(Trashnode node) {
    path.e_push_back(node, getmaxcapacity());
    evalLast();
}


void Vehicle::push_front(Trashnode node) {
    // position 0 is the depot we can not put a node before that
    path.e_insert(node, 1, getmaxcapacity());
    path.evaluate(1, getmaxcapacity());
    evalLast();
}


void Vehicle::insert(Trashnode node, int at) {
    path.e_insert(node, at, getmaxcapacity());
    path.evaluate(at, getmaxcapacity());
    evalLast();
}


void Vehicle::evalLast() {
    Trashnode last = path[path.size()-1];
    dumpsite.setdemand(-last.getcargo());
    dumpsite.evaluate(last, getmaxcapacity());
    backToDepot.evaluate(dumpsite, getmaxcapacity());
    cost = w1*backToDepot.gettotDist() +
           w2*backToDepot.getcvTot() +
           w3*backToDepot.gettwvTot();
}

//--------------------------------------------------------------------------
// intra-route optimiziation
//--------------------------------------------------------------------------

bool Vehicle::doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4) {
    // Feasible exchanges only
    if ( c3 == c1 || c3 == c2 || c4 == c1 || c4 == c2 || c2 < 1 || c3 < 2 )
        return false;

    double oldcost = getcost();

    // Leave values at positions c1, c4
    // Swap c2, c3
    // c3 -> c2
    // c2 -> c3
    // reverse any nodes between c2 and c3
    path.e_swap(c2, c3, getmaxcapacity());
    path.e_reverse(c2+1, c3-1, getmaxcapacity());
    evalLast();

    // if the change does NOT improve the cost or generates TW violations
    // undo the change
    if (getcost() > oldcost or hastwv()) {
        path.e_swap(c2, c3, getmaxcapacity());
        path.e_reverse(c2+1, c3-1, getmaxcapacity());
        evalLast();
        return false;
    }

    return true;
}


bool Vehicle::doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6) {
    // Feasible exchanges only
    if (! (c2>c1 && c3>c2 && c4>c3 && c5>c4 && c6>c5)) return false;

    double oldcost = getcost();
    Twpath<Trashnode> oldpath(path); // save a copy for undo

    // the 3-opt appears to reduce to extracting a sequence of nodes c3-c4
    // and reversing them and inserting them back after c6
//std::cout << "doThreeOpt A: "; dumppath();
    path.e_movereverse(c2, c3, c6, getmaxcapacity());
    evalLast();
//std::cout << "doThreeOpt B: "; dumppath();


//std::cout << "doThreeOpt: cost: " << getcost() << ", twv: " << hastwv() << std::endl;

    if (getcost() > oldcost or hastwv()) {
        path = oldpath;
        evalLast();
        return false;
    }

    return true;
}


bool Vehicle::doOrOpt(const int& c1, const int& c2, const int& c3) {
    // Feasible exchanges only
    if (! (c2 >= c1 and (c3 < c1-1 or c3 > c2+2))) return false;
    if (c2 > path.size()-1 or c3 > path.size()-1) return false;

    double oldcost = getcost();
    Twpath<Trashnode> oldpath(path); // save a copy for undo

    path.e_move(c1, c2, c3, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        //path.e_move(c3-(c2-c1+1), c3-1, c1, getmaxcapacity());
        path = oldpath;
        evalLast();
        return false;
    }

    return true;
}


bool Vehicle::doNodeMove(const int& i, const int& j) {
    if (i == j or i < 1 or j < 1 or i > path.size()-1 or j > path.size()-1)
        return false;

    double oldcost = getcost();

    path.e_move(i, j, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        if (i > j)
            path.e_move(j, i+1, getmaxcapacity());
        else
            path.e_move(j-1, i, getmaxcapacity());
        evalLast();
        return false;
    }

    return true;
}


bool Vehicle::doNodeSwap(const int& i, const int& j) {
    if (i < 1 or j < 1 or i > path.size()-1 or j > path.size()-1)
        return false;

    double oldcost = getcost();

    path.e_swap(i, j, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        path.e_swap(i, j, getmaxcapacity());
        evalLast();
        return false;
    }

    return true;
}


bool Vehicle::doInvertSeq(const int& i, const int& j) {
    if (i > path.size() or j > path.size()-1)
        return false;

    double oldcost = getcost();

    path.e_reverse(i, j, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        path.e_reverse(i, j, getmaxcapacity());
        evalLast();
        return false;
    }

    return true;
}


bool Vehicle::pathOptimize() {
    // repeat each move until the is no improvement then move to the next

    // 1. move node forward
    // 2. move node down
    pathOptMoveNodes();

    // 3. 2-exchange
    pathOptExchangeNodes();

    // 4. sequence invert
    pathOptInvertSequence();
}


bool Vehicle::pathTwoOpt() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    do {
        oldcost = getcost();

        for (int i=0; i<size-3; i++) {
            for (int j=i+3; j<size-1; j++) {
                doTwoOpt( i, i+1, j, j+1 );
//                std::cout << "pathTwoOpt["<<i<<","<<i+1<<","<<j<<","<<j+1<<"]("<<getcost()<<"): ";
//                dumppath();
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathThreeOpt() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    do {
        oldcost = getcost();

        for (int i=0; i<size-5; i++) {
            for (int j=i+2; j<size-3; j++) {
                for (int k=j+2; k<size-1; k++) {
                    doThreeOpt( i, i+1, j, j+1, k, k+1 );
//                    std::cout << "pathThreeOpt["<<i<<","<<i+1<<","<<j<<","<<j+1<<","<<k<<","<<k+1<<"]("<<getcost()<<"): ";
//                    dumppath();
                }
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOrOpt() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=1; j<size; j++) {
                for (int k=2; k>=0; k--) {
                    if (! (j<i-1 or j>i+k+1)) continue;
                    doOrOpt( i, i+k, j );
//std::cout << "pathOrOpt["<<i<<","<<i+k<<","<<j<<"]("<<getcost()<<"): ";
//dumppath();
                }
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOptMoveNodes() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    // move the nodes forward looking for improvements
    do {
        oldcost = getcost();

//std::cout << "\n---------------------------------------------------------\n";
//std::cout << "before move forward: "; dumppath();

        for (int i=1; i<size; i++) {
            for (int j=i+1; j<size; j++) {
                doNodeMove(i, j);
//std::cout << "after doNodeMove("<<i<<","<<j<<"): "; dumppath();
            }
        }
    }
    while (getcost() < oldcost);

    // move the nodes backwards looking for improvements
    do {
        oldcost = getcost();

//std::cout << "\n---------------------------------------------------------\n";
//std::cout << "before move backward: "; dumppath();

        for (int i=size-1; i>0; i--) {
            for (int j=i-1; j>0; j--) {
                doNodeMove(i, j);
//std::cout << "after doNodeMove("<<i<<","<<j<<"): "; dumppath();
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOptExchangeNodes() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    // 2-exchange (swapping) nodes along the path
    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=i+1; j<size; j++) {
                doNodeSwap(i, j);
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOptInvertSequence() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    // invert all possible sequences of nodes
    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=i+1; j<size; j++) {
                doInvertSeq(i, j);
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


// --------------------------------------------------------------------------
// Inter-route modifications
// --------------------------------------------------------------------------

/*
    2-exchange - swap path[i1] with v2[i2]
*/
bool Vehicle::swap(Vehicle& v2, const int& i1, const int& i2) {
    if (i1 < 0 or i1 > this->size()-1 or i2 < 0 or i2 > v2.size()-1)
        return false;

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    Twpath<Trashnode>& v2p = v2.getvpath();

    path.e_swap(i1, getmaxcapacity(), v2p, i2, v2.getmaxcapacity());
    evalLast();
    v2.evalLast();

    double newcost1 = getcost();
    double newcost2 = v2.getcost();

    if ( newcost1 + newcost2 > oldcost1 + oldcost2 or
         hastwv() or hascv() or v2.hastwv() or v2.hascv() ) {
        path.e_swap(i1, getmaxcapacity(), v2p, i2, v2.getmaxcapacity());
        evalLast();
        v2.evalLast();

        return false;
    }

    return true;
}


/*
    3-route node exchange - swap3
    path[i1] -> v2.path[i2] -> v3.path[i3] -> path[i1]
*/
bool Vehicle::swap3(Vehicle& v2, Vehicle& v3, const int& i1, const int& i2, const int& i3) {
    if ( i1 < 0 or i1 > this->size()-1 or
         i2 < 0 or i2 > v2.size()-1 or
         i2 < 0 or i3 > v3.size()-1 ) return false;

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();
    double oldcost3 = v3.getcost();

    path.e_swap(i1, getmaxcapacity(),
        v2.getvpath(), i2, v2.getmaxcapacity());
    v2.getvpath().e_swap(i2, v2.getmaxcapacity(),
        v3.getvpath(), i3, v3.getmaxcapacity());
    evalLast();
    v2.evalLast();
    v3.evalLast();

    double newcost1 = getcost();
    double newcost2 = v2.getcost();
    double newcost3 = v3.getcost();

    if ( newcost1 + newcost2 + newcost3 > oldcost1 + oldcost2 + oldcost3 or
         !feasable() or !v2.feasable() or !v3.feasable() ) {
        v2.getvpath().e_swap(i2, v2.getmaxcapacity(),
            v3.getvpath(), i3, v3.getmaxcapacity());
        path.e_swap(i1, getmaxcapacity(),
            v2.getvpath(), i2, v2.getmaxcapacity());
        evalLast();
        v2.evalLast();
        v3.evalLast();

        return false;
    }

    return true;
}


/*
    exchange seq - exchange
    exchange a sequence of nodes between two routes
    path[i1..j1] <--> v2.path[i2..j2]
*/
bool Vehicle::exchangeSeq(Vehicle& v2, const int& i1, const int& j1, const int& i2, const int& j2) {
    if ( j1 < i1 or j2 < i2 or i1 < 0 or i2 < 0 or
         j1 > this->size()-1 or j2 > v2.size()-1 ) return false;

    const Twpath<Trashnode> p1 = getvpath();      // get a copy
    const Twpath<Trashnode> p2 = v2.getvpath();   // get a copy

    Twpath<Trashnode>& v2p = v2.getvpath(); // get a reference to manipulate

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    int d1 = j1-i1+1;
    int d2 = j2-i2+1;

    // first swap the nodes in the min length of the seq length
    for (int n=0; n<std::min(d1, d2); n++) {
        path.e_swap(i1+n, getmaxcapacity(),
            v2p, i2+n, v2.getmaxcapacity());
    }

    // now if the seqs were different lengths move the remainder
    if (d1 > d2) {
        for (int n=0; n<d1-d2; n++) {
            if (i2+d2+n >= v2.size())
                v2p.e_push_back(path[i1+d2], v2.getmaxcapacity());
            else
                v2p.e_insert(path[i1+d2], i2+d2+n, v2.getmaxcapacity());
            path.e_remove(i1+d2, getmaxcapacity());
        }
    }
    else if (d2 > d1) {
        for (int n=0; n<d2-d1; n++) {
            if (i1+d1+n >= this->size())
                path.e_push_back(v2p[i2+d1], getmaxcapacity());
            else
                path.e_insert(v2p[i2+d1], i1+d1+n, getmaxcapacity());
            v2p.e_remove(i2+d1, v2.getmaxcapacity());
        }
    }

    evalLast();
    v2.evalLast();

    double newcost1 = getcost();
    double newcost2 = v2.getcost();

    if ( newcost1 + newcost2 > oldcost1 + oldcost2 or
             !feasable() or !v2.feasable() ) {
        setvpath(p1);
        v2.setvpath(p2);
        evalLast();
        v2.evalLast();

        return false;
    }

    return true;
}


/*
    exchange tails
    this swaps the seq of nodes from an index to the end of the path with
    another path and its given index
    exchange v1[i1...n1] <--> v2[i2..n2]
*/
bool Vehicle::exchangeTails(Vehicle& v2, const int& i1, const int& i2) {
    if ( i1 < 0 or i1 > this->size()-1 or
         i2 < 0 or i2 > v2.size()-1 ) return false;

    return exchangeSeq(v2, i1, this->size()-1, i2, v2.size()-1);
}


/*
    exchange3
*/
bool Vehicle::exchange3(Vehicle& v2, Vehicle& v3, const int& cnt, const int& i1, const int& i2, const int& i3) {
    if ( i1 < 0 or i1+cnt > this->size()-1 or
         i2 < 0 or i2+cnt > v2.size()-1 or
         i3 < 0 or i3+cnt > v3.size()-1 or cnt < 1) return false;

    Twpath<Trashnode>& v2p = v2.getvpath(); // get a reference to manipulate
    Twpath<Trashnode>& v3p = v3.getvpath(); // get a reference to manipulate

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();
    double oldcost3 = v3.getcost();

std::cout << "oldcost: "<<oldcost1<<"+"<<oldcost2<<"+"<<oldcost3<<"="
          << oldcost1 + oldcost2 + oldcost3 << std::endl; 

    for (int i=0; i<cnt; i++) {
        path.e_swap(i1+i, getmaxcapacity(),
            v2.getvpath(), i2+i, v2.getmaxcapacity());
        v2.getvpath().e_swap(i2+i, v2.getmaxcapacity(),
            v3.getvpath(), i3+i, v3.getmaxcapacity());
    }
    evalLast();
    v2.evalLast();
    v3.evalLast();

    double newcost1 = getcost();
    double newcost2 = v2.getcost();
    double newcost3 = v3.getcost();

std::cout << "newcost: "<<newcost1<<"+"<<newcost2<<"+"<<newcost3<<"="
          << newcost1 + newcost2 + newcost3 << std::endl; 

    if ( newcost1 + newcost2 + newcost3 > oldcost1 + oldcost2 + oldcost3 or
         !feasable() or !v2.feasable() or !v3.feasable() ) {
        for (int i=0; i<cnt; i++) {
            v2.getvpath().e_swap(i2+i, v2.getmaxcapacity(),
                v3.getvpath(), i3+i, v3.getmaxcapacity());
            path.e_swap(i1+i, getmaxcapacity(),
                v2.getvpath(), i2+i, v2.getmaxcapacity());
        }
        evalLast();
        v2.evalLast();
        v3.evalLast();

        return false;
    }

    return true;
}


/*
    relocate
    move node v1[i1] to another path v2[i2]
    returns false if it fails to relocate the node to v2
*/
bool Vehicle::relocate(Vehicle& v2, const int& i1, const int& i2) {
    if ( i1 < 0 or i1 > this->size()-1 or
         i2 < 0 or i2 > v2.size()-1 ) return false;

    Twpath<Trashnode>& v2p = v2.getvpath(); // get a reference to manipulate

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    v2p.e_insert(path[i1], i2, v2.getmaxcapacity());
    path.e_remove(i1, getmaxcapacity());

    evalLast();
    v2.evalLast();

    double newcost1 = getcost();
    double newcost2 = v2.getcost();

    if ( newcost1 + newcost2 > oldcost1 + oldcost2 or
         !feasable() or !v2.feasable() ) {
        path.e_insert(v2p[i2], i1, getmaxcapacity());
        v2p.e_remove(i2, v2.getmaxcapacity());
        evalLast();
        v2.evalLast();
        return false;
    }
    return true;
}


/*
    relocateBest
    move node v1[i1] to the best location in v2
    returns false if it fails to insert it.
*/
bool Vehicle::relocateBest(Vehicle& v2, const int& i1) {
    if ( i1 < 0 or i1 > this->size()-1 ) return false;

    Twpath<Trashnode>& v2p = v2.getvpath(); // get a reference to manipulate

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    int bestpos = -1;
    double bestcost;

    // first we insert v1[i1] into the start of v2
    // ie: pos: 1 after the depot at pos: 0
    v2p.e_insert(path[i1], 1, v2.getmaxcapacity());
    v2.evalLast();

    if (v2.feasable()) {
        bestpos = 1;
        bestcost = v2.getcost();
    }

    // now we walk it down the path checking for better costs
    for (int i=2; i<v2.size(); i++) {
        v2p.e_swap(i-1, i, v2.getmaxcapacity());
        v2.evalLast();
        if (v2.getcost() < bestcost and v2.feasable()) {
            bestpos = i;
            bestcost = v2.getcost();
        }
    }

    // if we found NO feasable place to insert it
    // restore v2 and return
    if (bestpos == -1) {
        v2p.e_remove(v2.size()-1, v2.getmaxcapacity());
        v2.evalLast();
        return false;
    }

    // otherwise remove i1
    path.e_remove(i1, getmaxcapacity());
    evalLast();

    // check that we have a better overall cost
    // and reposition i1 to bestpos
    if (getcost() + bestcost < oldcost1 + oldcost2) {
        if (bestpos != v2.size()-1) {
            v2p.e_move(v2.size()-1, bestpos, v2.getmaxcapacity());
            v2.evalLast();
        }
    }
    // else restore everything and return false
    else {
        path.e_insert(v2p[v2.size()-1], i1, getmaxcapacity());
        evalLast();
        v2p.e_remove(v2.size()-1, v2.getmaxcapacity());
        v2.evalLast();
        return false;
    }

    return true;
}



