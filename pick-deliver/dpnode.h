#ifndef DPNODE_H
#define DPNODE_H


#include "twnode.h"

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

    void copyvalues (const Dpnode &other);

public:
    bool ispickup() const {return hassupply();}
    bool isdepot() const {return hasdemand();}
    bool isdelivery() const {return  hasnogoods();}
    bool hastwv() const {return twv;}
    bool hascv() const {return cv;}
    void dumpeval();
/*accessors*/
    int  gettwvTot() const {return twvTot;}
    int  getcvTot() const {return cvTot;}
    double getcargo() const {return cargo;}
    double getdistPrev() const {return distPrev;};
    double gettotDist() const {return totDist;};
    
/* mutators */        
    void evaluate (double cargoLimit) ;
    void evaluate (const Dpnode &pred,double cargoLimit);  
/* constructors &destructors */
   Dpnode(){};
   ~Dpnode(){};

   Dpnode(Twnode &n);

    Dpnode(const Dpnode &other):Twnode(other) {
              copyvalues(other);
     };
    Dpnode& operator=(const Dpnode &other) {
              copyvalues(other);
     };        

};    

#endif
