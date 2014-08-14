

#include <iostream>


#include "path.h"
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
    std::cout << "maxcapacity: " << maxcapacity << std::endl;
    std::cout << "curcapacity: " << curcapacity << std::endl;
    std::cout << "duration: " << duration << std::endl;
    std::cout << "cost: " << cost << std::endl;
    std::cout << "TWV: " << TWV << std::endl;
    std::cout << "CV: " << CV << std::endl;
    std::cout << "w1: " << w1 << std::endl;
    std::cout << "w2: " << w2 << std::endl;
    std::cout << "w3: " << w3 << std::endl;
    std::cout << "path nodes: -----------------" << std::endl;
    Path::dump();
}

