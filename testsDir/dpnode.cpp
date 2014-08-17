#include <iostream>
#include <sstream>

#include "dpnode.h"

    
    void Dpnode::evaluate (double cargoLimit) {
        cargo=getdemand();
        waitTime=0;
        distPrev=0;
        totDist=0;
        }
        
    void Dpnode::evaluate (const Dpnode &pred,double cargoLimit){  
        distPrev=distance(pred);      //vehicle last move
        totDist=pred.gettotDist();
        twv=latearrival(totDist);     //Time Window Violation
             
        waitTime=earlyarrival(totDist)? opens()-totDist:0;
        totDist+=waitTime+getservicetime(); //totDist=opens()   should gice the same result

        cargo=pred.getcargo()+getdemand();       //loading or unloading 
        cv= cargo>cargoLimit or cargo < 0;  //capacity Violation
        twvTot = (twv)? pred.twvTot+pred.twvTot:pred.twvTot;
        cvTot = (cv)? pred.cvTot+1:pred.cvTot;
   };




    void Dpnode::dumpeval() {
        dump();
        std::cout<<"twv="<<twv
                 <<",cv="<<cv
                 <<",twvTot="<<twvTot
                 <<",cvTot="<<cvTot
                 <<",cargo="<<cargo
                 <<",distPrev="<<distPrev
                 <<",totDist="<<totDist
                 <<"\n";
    };

    void Dpnode::copyvalues (const Dpnode &other) {
              twv=other.twv;
              cv=other.cv;
              twvTot=other.twvTot;
              cvTot=other.cvTot;
              cargo=other.cargo;
              distPrev=other.distPrev;
              totDist=other.totDist;
             };


   Dpnode::Dpnode(Twnode &n):Twnode(n) {
              twv=false;
              cv=false;
              twvTot=0;
              cvTot=0;
              cargo=0;
              distPrev=0;
              totDist=0;
    };

