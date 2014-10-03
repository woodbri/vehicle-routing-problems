#ifndef TWEVAL_H
#define TWEVAL_H

#include <cassert>
#include <vector>
#include "twnode.h"

//to evaluate the vehicle at node level

class Tweval: public Twnode {
protected:


public:
    static std::vector<std::vector<double> > TravelTime;

    void dumpeval() const;
    void dump() const ;
/*accessors*/
    int  gettwvTot() const { return twvTot; };
    int  getcvTot() const { return cvTot; };
    double getcargo() const { return cargo; };
    double getdistPrev() const { return travelTime; };


    double getTotTime() const { return departureTime;};
    double getTravelTime() const { return travelTime; };
    double getArrivalTime() const { return arrivalTime; };
    double getWaitTime() const {return waitTime;};
    double getDepartureTime() const { return departureTime; };
    double getTotTravelTime() const { return totTravelTime; };
    double getTotWaitTime() const { return totWaitTime; };
    double getTotServiceTime() const { return totServiceTime; };
    

    bool hastwv() const { return twv; };
    bool hascv() const { return cv; };

/* mutators */        
    void evaluate (double cargoLimit);
    void evaluate (const Tweval &pred, double cargoLimit);  
    void setTravelTimes(const std::vector<std::vector<double> > &_tt) { 
        assert ( _tt.size() );
        TravelTime=_tt; 
        assert ( TravelTime.size() );
    }



/* Operators, to be discussed */


/* constructors &destructors */
    Tweval();
    Tweval(std::string line):Twnode(line){
        cv = twv = false;
	cvTot = twvTot = 0;
	cargo = 0;
	arrivalTime = travelTime = waitTime = departureTime =0;
        totWaitTime = totTravelTime = totServiceTime =0;
   };

    ~Tweval(){};

private:
    bool twv;
    bool cv;
    int twvTot;
    int cvTot;
    double cargo;

    double travelTime;
    double arrivalTime;
    double waitTime;
    //double serviceTime  already in twnode
    double departureTime;

    double totWaitTime;
    double totTravelTime;
    double totServiceTime;


};    


#endif
