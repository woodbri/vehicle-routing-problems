#ifndef DPNODE_H
#define DPNODE_H

#include <string>
#include "twnode.h"

class Dpnode: public Twnode {
private:
    //to know the other node in the order
    int oid;
    int did;
    int pid;
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
    void dump();
/*accessors*/
    int  gettwvTot() const {return twvTot;}
    int  getcvTot() const {return cvTot;}
    double getcargo() const {return cargo;}
    double getdistPrev() const {return distPrev;};
    double gettotDist() const {return totDist;};
    int getdid() const {return  did;};
    int getpid() const {return  pid;};
    int getoid() const {return oid;};
/* mutators */        
    void evaluate (double cargoLimit) ;
    void evaluate (const Dpnode &pred,double cargoLimit);  
/* constructors &destructors */

   Dpnode(std::string line);

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
