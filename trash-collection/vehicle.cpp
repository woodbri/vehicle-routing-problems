

#include <iostream>
#include <deque>

#include "twpath.h"
#include "vehicle.h"

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
    evalLast();
}



void Vehicle::insert(Trashnode node, int at) {
    path.insert(node, at, getmaxcapacity());
    evalLast();
}


void Vehicle::evaluate() {
    path.evaluate(getmaxcapacity());
};


void Vehicle::evaluate(int from) {
    Trashnode last = path[path.size()-1];
    dumpsite.evaluate(last, getmaxcapacity());
    backToDepot.evaluate(dumpsite, getmaxcapacity());
    cost = w1*backToDepot.gettotDist() +
           w2*backToDepot.getcvTot() +
           w3*backToDepot.gettwvTot();
}


void Vehicle::evalLast() {
    evaluate(path.size()-1);
}



/*
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

void Vehicle::dump() {
    std::cout << "---------- Vehicle ---------------" << std::endl;
    std::cout << "maxcapacity: " << getmaxcapacity() << std::endl;
    std::cout << "curcapacity: " << curcapacity << std::endl;
    std::cout << "duration: " << duration << std::endl;
    std::cout << "cost: " << cost << std::endl;
    std::cout << "TWV: " << TWV << std::endl;
    std::cout << "CV: " << CV << std::endl;
    std::cout << "w1: " << w1 << std::endl;
    std::cout << "w2: " << w2 << std::endl;
    std::cout << "w3: " << w3 << std::endl;
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



