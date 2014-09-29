#include <string>
#include <iostream>
#include <sstream>

#include "tweval.h"

    std::vector<std::vector<double> > Tweval::TravelTime;

    
    void Tweval::evaluate (double cargoLimit) {
        cargo = getdemand();
        waitTime = 0;
        travelTime = 0;
        totTime = 0;
        twvTot = cvTot = 0;
        twv = cv = false;
    }
        
    void Tweval::evaluate (const Tweval &pred, double cargoLimit){  
        assert(Tweval::TravelTime.size());

        travelTime = TravelTime[pred.nid][nid]; // Travel Time from previous node to this node
        totTime = pred.totTime + travelTime;    // tot length travel drom 1st node
        twv = latearrival(totTime); // Time Window Violation

        waitTime = earlyarrival(totTime) ? opens()-totTime : 0; // truck arrives before node opens, so waits 
        totTime += waitTime + serviceTime; // we add the waiting time + service time
        cargo = pred.cargo + demand; // loading truck demand>0 or unloading demand<0
        cv = cargo>cargoLimit or cargo <0; // capacity Violation

        // keep a total of violations
        twvTot = (twv) ? pred.twvTot+1 : pred.twvTot;
        cvTot =  (cv) ?  pred.cvTot+1 : pred.cvTot;
   }



    void Tweval::dump() const {
        Twnode::dump();
        std::cout << std::endl;;
    }

    void Tweval::dumpeval() const  {
        std::cout << "twv=" << twv
                  << ", cv=" << cv
                  << ", twvTot=" << twvTot
                  << ", cvTot=" << cvTot
                  << ", cargo=" << cargo
                  << ", travel Time=" << travelTime
                  << ", waitTime=" << waitTime
                  << ", serviceTime=" << serviceTime
                  << ", totTime=" << totTime
                  << std::endl;
    }



   Tweval::Tweval():Twnode() {
       twv = cv = false;
       twvTot = cvTot = 0;
       cargo = 0;
       waitTime = 0; 
       travelTime = totTime = 0;
    }


/* Private */
    void Tweval::copyvalues (const Tweval &other) {
              twv = other.twv;
              cv = other.cv;
              twvTot = other.twvTot;
              cvTot = other.cvTot;
              cargo = other.cargo;
              travelTime = other.travelTime;
              totTime = other.totTime;
              waitTime = other.waitTime;
    }


