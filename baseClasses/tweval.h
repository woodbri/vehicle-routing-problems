/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
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
    bool feasable() const { return twvTot == 0 and cvTot == 0;};


    double getTotTime() const { return departureTime;};
    double getTravelTime() const { return travelTime; };
    double getArrivalTime() const { return arrivalTime; };
    double getWaitTime() const {return waitTime;};
    double getDepartureTime() const { return departureTime; };
    double getTotTravelTime() const { return totTravelTime; };
    double getTotWaitTime() const { return totWaitTime; };
    double getTotServiceTime() const { return totServiceTime; };
    double getDumpVisits() const { return dumpVisits; };
    double deltaGeneratesTWV(double deltaTime) const;
    

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
    Tweval(std::string line):Twnode(line) {
        cv = twv = false;
        cvTot = twvTot = 0;
        cargo = 0;
        arrivalTime = travelTime = waitTime = departureTime =0;
        totWaitTime = totTravelTime = totServiceTime =0;
    };

    Tweval(int _id, double _x, double _y, int _open, int _close, int _service, int _demand, int _sid) : Twnode() {
        set(_id, _id, _x, _y, _demand, _open, _close, _service);
        setStreetId(_sid);
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
    double dumpVisits;


};    


#endif
