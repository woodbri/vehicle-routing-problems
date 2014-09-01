

#include <iostream>
#include <deque>

#include "twpath.h"
#include "vehicle.h"


void Vehicle::dump() {
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


void Vehicle::dumppath() {
    path.dump();
}


std::deque<int> Vehicle::getpath()  {
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


// found 2-opt and 3-opt algorithm here
// http://www.technical-recipes.com/2012/applying-c-implementations-of-2-opt-to-travelling-salesman-problems/
// but I do not think either the 2-opt of 3-opt as implemented are correct
// I have modified the 2-opt to reverse the intervening nodes
// which I believe corrects the 2-opt algorithm

void Vehicle::doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4) {
    // Feasible exchanges only
    if ( c3 == c1 || c3 == c2 || c4 == c1 || c4 == c2 || c2 < 1 || c3 < 2 )
        return;

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
    }

}


void Vehicle::doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6) {
    // Feasible exchanges only
    if (! (c2>c1 && c3>c2 && c4>c3 && c5>c4 && c6>c5)) return;

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
    }

}


void Vehicle::doOrOpt(const int& c1, const int& c2, const int& c3) {
    // Feasible exchanges only
    if (! (c2 >= c1 and (c3 < c1-1 or c3 > c2+2))) return;
    if (c2 > path.size()-1 or c3 > path.size()-1) return;

    double oldcost = getcost();
    Twpath<Trashnode> oldpath(path); // save a copy for undo

    path.e_move(c1, c2, c3, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        //path.e_move(c3-(c2-c1+1), c3-1, c1, getmaxcapacity());
        path = oldpath;
        evalLast();
    }
}


void Vehicle::doNodeMove(const int& i, const int& j) {
    if (i == j or i < 1 or j < 1 or i > path.size() or j > path.size())
        return;

    double oldcost = getcost();

    path.e_move(i, j, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        if (i > j)
            path.e_move(j, i+1, getmaxcapacity());
        else
            path.e_move(j-1, i, getmaxcapacity());
        evalLast();
    }
}


void Vehicle::doNodeSwap(const int& i, const int& j) {
    if (i < 1 or j < 1 or i > path.size() or j > path.size()) return;

    double oldcost = getcost();

    path.e_swap(i, j, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        path.e_swap(i, j, getmaxcapacity());
        evalLast();
    }
}


void Vehicle::doInvertSeq(const int& i, const int& j) {
    if (i > path.size() or j > path.size()) return;

    double oldcost = getcost();

    path.e_reverse(i, j, getmaxcapacity());
    evalLast();

    if (getcost() > oldcost or hastwv()) {
        path.e_reverse(i, j, getmaxcapacity());
        evalLast();
    }
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



