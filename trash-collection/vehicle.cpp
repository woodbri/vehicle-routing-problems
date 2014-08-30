

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
    path.push_back(node, getmaxcapacity());
    evalLast();
}


void Vehicle::push_front(Trashnode node) {
    // position 0 is the depot we can not put a node before that
    path.insert(node, 1, getmaxcapacity());
    path.evaluate(1, getmaxcapacity());
    evalLast();
}


void Vehicle::insert(Trashnode node, int at) {
    path.insert(node, at, getmaxcapacity());
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


void Vehicle::doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4) {
    // Feasible exchanges only
    if ( c3 == c1 || c3 == c2 || c4 == c1 || c4 == c2 ) return;

    double oldcost = getcost();

    // Leave values at positions c1, c4
    // Swap c2, c3
    // c3 -> c2
    // c2 -> c3
    path.swap(c2, c3, getmaxcapacity());
    evalLast();

    // if the change does NOT improve the cost or generates TW violations
    // undo the change
    if (getcost() > oldcost or hastwv()) {
        path.swap(c3, c2, getmaxcapacity());
        evalLast();
    }

}


void Vehicle::doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6) {
    // Feasible exchanges only - TODO not sure what we should eliminate

    double oldcost = getcost();

    // swap 1
    // c2 -> c4
    // c4 -> c6
    // c6 -> c2
    // before: 1 2 3 4 5 6
    // after:  1 4 3 6 5 2
    path.swap(c4, c6, getmaxcapacity());
    path.swap(c2, c4, getmaxcapacity());
    evalLast();

    // reset if not an improvement
    if (getcost() > oldcost or hastwv()) {
        path.swap(c4, c2, getmaxcapacity());
        path.swap(c6, c4, getmaxcapacity());
        evalLast();
    }
    else {
        oldcost = getcost();
    }

    // swap 2
    // c2 -> c4
    // c4 -> c5
    // c5 -> c2
    // before: 1 2 3 4 5 6
    // after:  1 5 3 2 4 6
    path.swap(c4, c5, getmaxcapacity());
    path.swap(c2, c4, getmaxcapacity());
    evalLast();

    // reset if not an improvement
    if (getcost() > oldcost or hastwv()) {
        path.swap(c4, c2, getmaxcapacity());
        path.swap(c5, c4, getmaxcapacity());
        evalLast();
    }
}


bool Vehicle::pathOptimize() {

}


bool Vehicle::pathTwoOpt() {
    int size = this->size();

    double oldcost = getcost();

    for (int i=0; i<size-3; i++) {
        for (int j=i+3; j<size-1; j++) {
            doTwoOpt( i, i+1, j, j+1 );
        }
    }

    double newcost = getcost();

    return newcost < oldcost;
}


bool Vehicle::pathThreeOpt() {
    int size = this->size();

    double oldcost = getcost();

    for (int i=0; i<size-5; i++) {
        for (int j=i+3; j<size-3; j++) {
            for (int k=j+3; k<size-1; k++) {
                doThreeOpt( i, i+1, j, j+1, k, k+1 );
            }
        }
    }

    double newcost = getcost();

    return newcost < oldcost;
}


/*
// this is the old code before we started using Tweval
void Vehicle::evaluate() {
    curcapacity = 0;
    duration = 0;
    cost = 0;
    TWV = 0;
    CV = 0;

    if (path.size()) {
        for (int i=0; i<path.size(); i++) {
            if (i == 0)
                duration += distancetodepot(i);
            else
                duration += path[i].distance(path[i-1]);

            if (path[i].earlyarrival(duration))
                duration = path[i].opens();

            if (path[i].latearrival(duration)) 
                TWV++;

            duration += path[i].getservicetime();

            curcapacity += path[i].getdemand();
            if (curcapacity > getmaxcapacity())
                CV++;
        }

        duration += getdumpsite().distance(path.back());

        if (getdumpsite().earlyarrival(duration))
            duration = getdumpsite().opens();

        if (getdumpsite().latearrival(duration))
            TWV++;

        duration += getdumpsite().getservicetime();

        duration += getdumpsite().distance(getdepot());
        if (getdepot().latearrival(duration))
            TWV++;

    }
    cost = w1*duration + w2*TWV +w3*CV;
}
*/
