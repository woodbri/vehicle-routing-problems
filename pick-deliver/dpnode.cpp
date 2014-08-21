#include <string>
#include <iostream>
#include <sstream>

#include "dpnode.h"

    
    void Dpnode::evaluate (double cargoLimit) {
        cargo=getdemand();
        waitTime=0;
        distPrev=0;
        totDist=0;
        twvTot=cvTot=0;
        twv=cv=false;
        }
        
    void Dpnode::evaluate (const Dpnode &pred,double cargoLimit){  
//std::cout<<"cargo Limit\n"<<cargoLimit;

        distPrev=distance(pred);       //vehicle last move length
        totDist=pred.totDist+distPrev; //tot length travel drom 1st node
        twv=latearrival(totDist);      //Time Window Violation

        waitTime=earlyarrival(totDist)? opens()-totDist:0;   //truck arrives before node opens, so waits 
        totDist+=waitTime+service;                          // we add the waiting time + service time
//std::cout<<"previus\n";
//pred.dumpeval();
//std::cout<<"actual\n";
//dumpeval();
        cargo=pred.cargo+getdemand();                       //loading truck demand>0 or unloading demand<0
        cv= cargo>cargoLimit or cargo <0;                   //capacity Violation
        twvTot = (twv)? pred.twvTot+1:pred.twvTot;          //keep a total of violations
        cvTot =  (cv)?  pred.cvTot+1 :pred.cvTot;
   };



    void Dpnode::dump() const {
        Twnode::dump();
        std::cout<<"\t "<<pid
                 <<"\t "<<did;
       if (ispickup()) std::cout<<"\tpickup:"<<demand;
       if (isdelivery()) std::cout<<"\tdelivery:"<<demand;
       std::cout<<"\n";
        }

    void Dpnode::dumpeval() const  {
        dump();
        std::cout<<"twv="<<twv
                 <<",cv="<<cv
                 <<",twvTot="<<twvTot
                 <<",cvTot="<<cvTot
                 <<",cargo="<<cargo
                 <<",distWithPrev="<<distPrev
                 <<",waitTime="<<waitTime
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
              pid=other.pid;
              did=other.did;
              oid=other.oid;
             };


/*   Dpnode::Dpnode(Twnode &n):Twnode(n) {
              twv=false;
              cv=false;
              twvTot=0;
              cvTot=0;
              cargo=0;
              distPrev=0;
              totDist=0;
              oid=did=pid=0;
    };
*/
Dpnode::Dpnode(std::string line) {

    std::istringstream buffer( line );
    buffer >> nid;
    buffer >> x;
    buffer >> y;
    buffer >> demand;
    buffer >> tw_open;
    buffer >> tw_close;
    buffer >> service;
    buffer >> pid;
    buffer >> did;
    waitTime=0;
    distPrev=0;
    totDist=0;
    twvTot=cvTot=0;
    twv=cv=false;
//std::cout<<"\njust read:"; dump();
}

