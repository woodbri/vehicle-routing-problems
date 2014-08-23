#include <string>
#include <iostream>
#include <sstream>

#include "tweval.h"

    
    void Tweval::evaluate (double cargoLimit) {
        cargo=getdemand();
        waitTime=0;
        distPrev=0;
        totDist=0;
        twvTot=cvTot=0;
        twv=cv=false;
        }
        
    void Tweval::evaluate (const Tweval &pred,double cargoLimit){  

        distPrev=distance(pred);       //vehicle last move length
        totDist=pred.totDist+distPrev; //tot length travel drom 1st node
        twv=latearrival(totDist);      //Time Window Violation

        waitTime=earlyarrival(totDist)? opens()-totDist:0;   //truck arrives before node opens, so waits 
        totDist+=waitTime+service;                          // we add the waiting time + service time
        cargo=pred.cargo+getdemand();                       //loading truck demand>0 or unloading demand<0
        cv= cargo>cargoLimit or cargo <0;                   //capacity Violation
        twvTot = (twv)? pred.twvTot+1:pred.twvTot;          //keep a total of violations
        cvTot =  (cv)?  pred.cvTot+1 :pred.cvTot;
   };



    void Tweval::dump() const {
        Twnode::dump();
        std::cout<<"\n";
        }

    void Tweval::dumpeval() const  {
        std::cout<<"twv="<<twv
                 <<",cv="<<cv
                 <<",twvTot="<<twvTot
                 <<",cvTot="<<cvTot
                 <<",cargo="<<cargo
                 <<",distWithPrev="<<distPrev
                 <<",waitTime="<<waitTime
                 <<",serviceTime="<<service
                 <<",totDist="<<totDist
                 <<"\n";
    };



   Tweval::Tweval():Twnode() {
       twv= cv=false;
       twvTot= cvTot=0;
       cargo=0; waitTime=0; 
       distPrev= totDist=0;
    };


/* Private */
    void Tweval::copyvalues (const Tweval &other) {
              twv=other.twv;
              cv=other.cv;
              twvTot=other.twvTot;
              cvTot=other.cvTot;
              cargo=other.cargo;
              distPrev=other.distPrev;
              totDist=other.totDist;
              waitTime=other.waitTime;
    };


