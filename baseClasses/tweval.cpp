#include <string>
#include <iostream>
#include <sstream>

#include "tweval.h"

    std::vector<std::vector<double> > Tweval::TravelTime;

    
    void Tweval::evaluate (double cargoLimit) {
        cargo = demand;
        travelTime=0;
        arrivalTime= opens();
        totTravelTime = 0;
        totWaitTime = 0;
        totServiceTime= serviceTime;
        departureTime = arrivalTime + serviceTime;
	dumpVisits=type==1? 1 :  0;
        twvTot = cvTot = 0;
        twv = cv = false;
    }
        

    void Tweval::evaluate (const Tweval &pred, double cargoLimit){  
        assert(Tweval::TravelTime.size());
	
        travelTime     = TravelTime[pred.nid][nid]; // Travel Time from previous node to this node
        totTravelTime  = pred.totTravelTime + travelTime;    // tot length travel from 1st node

        arrivalTime   = pred.departureTime + travelTime; // Time Window Violation
        twv = latearrival(arrivalTime); // Time Window Violation

        waitTime     = earlyarrival(arrivalTime) ? opens()-arrivalTime : 0; // truck arrives before node opens, so waits 
        totWaitTime  = pred.totWaitTime + waitTime; 

        totServiceTime = pred.totServiceTime + serviceTime;
        departureTime      = arrivalTime + waitTime + serviceTime; 

	if (type==1 and pred.cargo>=0) demand= - pred.cargo;   //type 1 empties the truck (aka dumpSite)
	dumpVisits=type==1? pred.dumpVisits+1 :  pred.dumpVisits;

        cargo = pred.cargo + demand;// loading truck demand>0 or unloading demand<0
        cv = cargo>cargoLimit or cargo <0; // capacity Violation

        // keep a total of violations
        twvTot = (twv) ? pred.twvTot+1 : pred.twvTot;
        cvTot =  (cv) ?  pred.cvTot+1 : pred.cvTot;
   }



    void Tweval::dump() const {
        Twnode::dump();
        std::cout << std::endl;
    }

    void Tweval::dumpeval() const  {
        std::cout << "twv=" << twv
                  << ", cv=" << cv
                  << ", twvTot=" << twvTot
                  << ", cvTot=" << cvTot
                  << ", cargo=" << cargo
                  << ", travel Time=" << travelTime
                  << ", arrival Time=" << arrivalTime
                  << ", wait Time=" << waitTime
                  << ", service Time=" << serviceTime
                  << ", departure Time=" << departureTime
                  << std::endl;
    }



   Tweval::Tweval():Twnode() {
        arrivalTime = waitTime =  travelTime = 0;
        totTravelTime = totWaitTime = totServiceTime= 0;
        twvTot = cvTot = 0;
        twv = cv = false;
   }



