#ifndef TWEVAL_H
#define TWEVAL_H

#include "twnode.h"

//to evaluate the vehicle at node level

class Tweval: public Twnode {
public:
    void dumpeval() const;
    void dump() const ;
/*accessors*/
    int  gettwvTot() const { return twvTot; };
    int  getcvTot() const { return cvTot; };
    double getcargo() const { return cargo; };
    double getdistPrev() const { return distPrev; };
    double gettotDist() const { return totDist; };
    bool hastwv() const { return twv; };
    bool hascv() const { return cv; };

/* mutators */        
    void evaluate (double cargoLimit);
    void evaluate (const Tweval &pred, double cargoLimit);  

/* Operators, to be discussed */


/* constructors &destructors */
    Tweval();
    Tweval(std::string line):Twnode(line){};
    ~Tweval(){};

private:
    bool twv;
    bool cv;
    int twvTot;
    int cvTot;
    double cargo;
    double waitTime;
    double distPrev;
    double totDist;;

    void copyvalues (const Tweval &other);

};    

#endif
