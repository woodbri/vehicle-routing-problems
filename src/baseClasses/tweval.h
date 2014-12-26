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
#ifndef SRC_BASECLASSES_TWEVAL_H_
#define SRC_BASECLASSES_TWEVAL_H_

// #include <vector>
#include <string>

// #include "vrp_assert.h"
#include "./twnode.h"
#include "./twc.h"


/*! \class Tweval
 * \brief Extend Twnode to evaluate the vehicle at node level
 *
 * This class extends Twnode by adding attributes to store information
 * about the node in a path and provides the tools evaluate the node
 * and to set and get these attribute values.
 */


class Tweval: public Twnode {
 public:
    void dumpeval(double cargoLimit) const;
    void dump() const;
    /*accessors*/
    double travelTime() const { return travelTime_; }
    double arrivalTime() const { return arrivalTime_; }
    double waitTime() const {return waitTime_;}
    double departureTime() const { return departureTime_; }
    double deltaTime() const { return deltaTime_; }

    int  twvTot() const { return twvTot_; }
    int  cvTot() const { return cvTot_; }
    double cargo() const { return cargo_; }
    double getTotTime() const { return departureTime_;}
    double totTravelTime() const { return totTravelTime_; }
    double totWaitTime() const { return totWaitTime_; }
    double totServiceTime() const { return totServiceTime_; }
    int dumpVisits() const { return dumpVisits_; }


    double deltaGeneratesTWV(double deltaTime) const;
    bool feasable() const { return twvTot_ == 0 and cvTot_ == 0;}
    bool has_twv() const {
      return twvTot_ > 0 or lateArrival( arrivalTime_ );
    }
    bool has_cv(double cargoLimit) const {
      return cargo_ > cargoLimit or cargo_ < 0;
    }

    /* mutators */
    void evaluate(double cargoLimit);
    void evaluate(const Tweval &pred, double cargoLimit);


    /* constructors &destructors */

    Tweval();
    explicit Tweval(std::string line);
    Tweval(int id, double x, double y, int opens, int closes,
           int serviceTime, int demand, int streetId);

 private:
    double travelTime_;      ///< Travel time from last node
    double arrivalTime_;     ///< Arrival time at this node
    double waitTime_;        ///< Wait time at this node is early arrival
    double departureTime_;   ///< Departure time from this node
    double deltaTime_;   ///< Departure time from this node

    double cargo_;           ///< Total accumulated cargo at this point in the path
    int twvTot_;             ///< Total count of TWV at this point in the path
    int cvTot_;              ///< Total count of CV at this point in the path
    double totWaitTime_;     ///< Total accumulated wait time at this point in the path
    double totTravelTime_;   ///< Total accumulated travel time at this point in the path
    double totServiceTime_;  ///< Total accumulated service time at this point in the path
    int dumpVisits_;      ///< Total count of dump visits at this point in the path
};


#endif  // SRC_BASECLASSES_TWEVAL_H_
