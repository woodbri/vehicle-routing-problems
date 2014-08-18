

#include <iostream>


#include "vehicle.h"

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
    Twpath::dump();
}


void Vehicle::plot( std::vector<double> &x, std::vector<double> &y,
                    std::vector<int> &label, std::vector<int> &color ) {
    for (int i=0; i<path.size(); i++) {
        x.push_back(path[i].getx());
        y.push_back(path[i].gety());
        label.push_back(path[i].getnid());
        if (path[i].isdepot()) color.push_back(0xff0000);
        else if (path[i].ispickup()) color.push_back(0x00ff00);
        else if (path[i].isdump()) color.push_back(0x0000ff);
        else color.push_back(0xffff00);
    }
}



