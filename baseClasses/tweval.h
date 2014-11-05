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
    double deltaGeneratesTWV( double deltaTime ) const;


    bool hastwv() const { return twv; };
    bool hascv() const { return cv; };

    /* mutators */
    void evaluate ( double cargoLimit );
    void evaluate ( const Tweval &pred, double cargoLimit );

    /*!
     * \brief Assign a travel time matrix to the class.
     *
     * The travel time matrix is a static class object that is shared
     * between all the Tweval nodes.
     *
     * \param[in] _tt A reference to a travel time matrix.
     */
    void setTravelTimes( const std::vector<std::vector<double> > &_tt ) {
        assert ( _tt.size() );
        TravelTime = _tt;
        assert ( TravelTime.size() );
    }



    /* Operators, to be discussed */


    /* constructors &destructors */

    /*!
     * \brief Default constructor.
     */
    Tweval();

    /*!
     * \brief Construct a node from a text string, typically read from a file.
     */
    Tweval( std::string line ): Twnode( line ) {
        cv = twv = false;
        cvTot = twvTot = 0;
        cargo = 0;
        arrivalTime = travelTime = waitTime = departureTime = 0;
        totWaitTime = totTravelTime = totServiceTime = 0;
    };

    /*!
     * \brief Construct a node from arguments
     * \param[in] _id The User node id
     * \param[in] _x The X or longitude coordinate for its location
     * \param[in] _y The Y or latitude coordinate for its location
     * \param[in] _open The earliest arrival time (TW open)
     * \param[in] _close The latest arrival time (TW close)
     * \param[in] _service The service time
     * \param[in] _demand The demand in units of vehicle capacity
     * \param[in] _sid The street id this node is located
     */
    Tweval( int _id, double _x, double _y, int _open, int _close, int _service,
            int _demand, int _sid ) : Twnode() {
        set( _id, _id, _x, _y, _demand, _open, _close, _service );
        setStreetId( _sid );
        cv = twv = false;
        cvTot = twvTot = 0;
        cargo = 0;
        arrivalTime = travelTime = waitTime = departureTime = 0;
        totWaitTime = totTravelTime = totServiceTime = 0;
    };

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
    //double serviceTime  already in twnode
    double departureTime;   ///< Departure time from this node

    double totWaitTime;     ///< Total accumulated wait time at this point in the path
    double totTravelTime;   ///< Total accumulated travel time at this point in the path
    double totServiceTime;  ///< Total accumulated service time at this point in the path
    double dumpVisits;      ///< Total count of dump visits at this point in the path


};


#endif
