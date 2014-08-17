#ifndef PATHNODE_H
#define PATHNODE_H


#include "dpnode.h"

class Dpnode: public Twnode {
private:
    //to evaluate the vehicle at node level
    bool twv;
    bool cv;
    int twvTot;
    int cvTot;
    double cargo;
    double waitTime;
    double distPrev;
    double totDist;;

public:
    bool ispickup() const {return hassupply();}
    bool isdepot() const {return hasdemand();}
    bool isdelivery() const {return  hasnogoods();}

    bool hastwv() const {return twv;}
    bool hascv() const {return cv;}
    int  gettwvTot() const {return twvTot;}
    int  getcvTot() const {return cvTot;}
    double getcargo() const {return cargo;}
    double getdistPrev() const {return distPrev;};
    double gettotDist() const {return totDist;};
    
    void evaluate (double cargoLimit) {
        cargo=getdemand();
        waitTime=0;
        distPrev=0;
        totDist=0;
        }
        
    void evaluate (const Dpnode &pred,double cargoLimit){  
        distPrev=distance(pred);      //vehicle last move
        totDist=pred.gettotDist();
        twv=lateArrival(totDist);     //Time Window Violation
             
        waitTime=earlyArrival(totDist)? opens()-totDist:0;
        totDist+=waitTime+getServiceTime(); //totDist=opens()   should gice the same result

        cargo=pred.getcargo()+getdemand();       //loading or unloading 
        cv= cargo>cargoLimit or cargo < 0;  //capacity Violation
        twvTot = (twv)? pred.twvTot+pred.twvTot:pred.twvTot;
        cvTot = (cv)? pred.cvTot+1:pred.cvTot;
   };




    void dumpeval() {
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
/***********************/     
    void copyvalues (const Dpnode &other) {
              twv=other.twv;
              cv=other.cv;
              twvTot=other.twvTot;
              cvTot=other.cvTot;
              cargo=other.cargo;
              distPrev=other.distPrev;
              totDist=other.totDist;
             };

   Dpnode(){};

   Dpnode(Twnode &n):Twnode(n) {
              twv=false;
              cv=false;
              twvTot=0;
              cvTot=0;
              cargo=0;
              distPrev=0;
              totDist=0;
    };
    //pathNode(const pathNode &other):node(other.node) {
    Dpnode(const Dpnode &other):Twnode(other) {
              copyvalues(other);
     };
    Dpnode& operator=(const Dpnode &other) {
              copyvalues(other);
     };        

    ~Dpnode(){};
};    

#endif
