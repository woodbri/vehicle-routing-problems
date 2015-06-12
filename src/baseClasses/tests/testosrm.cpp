/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>

#include "timer.h"
#include "stats.h"

#include "osrmclient.h"
#include "containers.h"


int main(int argc, char **argv)
{

    int order[] = {1,2,3,4,5,
                   1,2,4,3,5,
                   1,3,2,4,5,
                   1,3,4,2,5,
                   1,4,2,3,5,
                   1,4,3,2,5};

    osrmi->useOsrm( true );
    for (int i=0; i<30; i+=5) {
        osrmi->clear();
        for (int j=0; j<5; j++)
            osrmi->addViaPoint( containers[order[i+j]].lat, containers[order[i+j]].lon );

        Timer osrmtime;
        bool ret = osrmi->getOsrmViaroute();
        STATS->addto("OSRM time", osrmtime.duration());

        if ( ret ) {

            std::deque<double> times;
            Timer parsetime;
            ret = osrmi->getOsrmTimes( times );
            STATS->addto("Parse time", parsetime.duration());
            if ( ret ) {
                std::cout << "Times:" << std::endl;
                for (int i=0; i<times.size(); i++)
                    std::cout << "i: " << i << ", time: " << times[i] << std::endl;
            }
            else {
                std::cout << "getOsrmTimes Failed!" << std::endl;
                return 1;
            }
        }
        else {
            std::cout << "getOsrmViaroute Failed!" << std::endl;
            return 1;
        }
    }

    osrmi->clear();
    // load all paths into a single route
    for (int i=0; i<30; i++)
        osrmi->addViaPoint( containers[order[i]].lat, containers[order[i]].lon );
    Timer osrmtime;
    bool ret = osrmi->getOsrmViaroute();
    STATS->addto("OSRM time2", osrmtime.duration());

    if ( ret ) {

        std::deque<double> times;
        Timer parsetime;
        ret = osrmi->getOsrmTimes( times );
        STATS->addto("Parse time2", parsetime.duration());
        if ( ret ) {
            std::cout << "Times:" << std::endl;
            for (int i=0; i<times.size(); i++)
                std::cout << "i: " << i << ", time: " << times[i] << std::endl;
        }
        else {
            std::cout << "getOsrmTimes Failed!" << std::endl;
            return 1;
        }
    }
    else {
        std::cout << "getOsrmViaroute Failed!" << std::endl;
        return 1;
    }

    STATS->dump("Stats:");

    return 0;
}
