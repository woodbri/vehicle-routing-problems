#include <string>
#include <iostream>
#include <sstream>

#include "tweval.h"

    
    void Tweval::evaluate (double cargoLimit) {
        cargo = getdemand();
        waitTime = 0;
        distPrev = 0;
        totDist = 0;
        twvTot = cvTot = 0;
        twv = cv = false;
    }
        
    void Tweval::evaluate (const Tweval &pred, double cargoLimit){  

        // vehicle last move length
        distPrev = distance(pred);
        // tot length travel drom 1st node
        totDist = pred.totDist + distPrev;
        // Time Window Violation
        twv = latearrival(totDist);

        // truck arrives before node opens, so waits 
        waitTime = earlyarrival(totDist) ? opens()-totDist : 0;
        // we add the waiting time + service time
        totDist += waitTime + service;
        // loading truck demand>0 or unloading demand<0
        cargo = pred.cargo + getdemand();
        // capacity Violation
        cv = cargo>cargoLimit or cargo <0;
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
                  << ", distWithPrev=" << distPrev
                  << ", waitTime=" << waitTime
                  << ", serviceTime=" << service
                  << ", totDist=" << totDist
                  << std::endl;
    }



   Tweval::Tweval():Twnode() {
       twv = cv = false;
       twvTot = cvTot = 0;
       cargo = 0;
       waitTime = 0; 
       distPrev = totDist = 0;
    }


/* Private */
    void Tweval::copyvalues (const Tweval &other) {
              twv = other.twv;
              cv = other.cv;
              twvTot = other.twvTot;
              cvTot = other.cvTot;
              cargo = other.cargo;
              distPrev = other.distPrev;
              totDist = other.totDist;
              waitTime = other.waitTime;
    }


