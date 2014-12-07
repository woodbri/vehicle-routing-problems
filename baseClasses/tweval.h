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

#include <vector>
#include <string>

#include "vrp_assert.h"
#include "twnode.h"
#include "twc.h"

#ifdef WITHOSRM
#include "vrposrm.h"
#endif

/*! \class Tweval
 * \brief Extend Twnode to evaluate the vehicle at node level
 *
 * This class extends Twnode by adding attributes to store information
 * about the node in a path and provides the tools evaluate the node
 * and to set and get these attribute values.
 */


class Tweval: public Twnode {
  protected:


  public:
    //static std::vector<std::vector<double> > TravelTime;

    void dumpeval() const;
    void dump() const ;
    /*accessors*/
    int  gettwvTot() const { return twvTot; };
    int  getcvTot() const { return cvTot; };
    double getCargo() const { return cargo; };
    double getDistPrev() const { return travelTime; };
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
    double deltaGeneratesTWV( double deltaTime ) const;
    std::string getLoc() const;


    bool hastwv() const { return twv; };
    bool hascv() const { return cv; };

    /* mutators */
    void evaluate ( double cargoLimit );
    void evaluate ( const Tweval &pred, double cargoLimit );

    /* Osrm stuff */
    #ifdef WITHOSRM
    void evaluateOsrm ();
    void evaluateOsrm ( const Tweval &pred, const std::string &osrmBaseUrl );
    bool isOsrmTtimeValid() { return getTotTravelTimeOsrm() == -1 ? false : true; };
    double getTotTravelTimeOsrm() const { return totTravelTimeOsrm; };
    std::string getOsrmUrlLocs() const { return osrmUrlLocs; };
    std::string getOsrmUrl( const std::string osrmBaseUrl ) const;
    #endif


    /* constructors &destructors */

    Tweval();
    Tweval( std::string line );
    Tweval( int _id, double _x, double _y, int _open, int _close,
            int _service, int _demand, int _sid );
    ~Tweval() {};

  private:
    bool twv;               ///< Has time window violations (TWV)
    bool cv;                ///< Has capacity violations (CV)
    int twvTot;             ///< Total count of TWV at this point in the path
    int cvTot;              ///< Total count of CV at this point in the path
    double cargo;           ///< Total accumulated cargo at this point in the path

    double travelTime;      ///< Travel time from last node
    double arrivalTime;     ///< Arrival time at this node
    double waitTime;        ///< Wait time at this node is early arrival
    double departureTime;   ///< Departure time from this node

    double totWaitTime;     ///< Total accumulated wait time at this point in the path
    double totTravelTime;   ///< Total accumulated travel time at this point in the path
    double totServiceTime;  ///< Total accumulated service time at this point in the path
    double dumpVisits;      ///< Total count of dump visits at this point in the path
    #ifdef WITHOSRM
    double totTravelTimeOsrm;   ///< Total accumulated travel time at this point in the path based on the OSRM path
    std::string
    osrmUrlLocs;    ///< The partial URL string to get OSRM traveltime to this position in the path
    #endif

};


#endif
