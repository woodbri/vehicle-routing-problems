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
  /** @name Dumps */
  ///@{
  /*! \brief Prints the evaluation of the truck */
  void dumpeval(double cargoLimit) const;
  /*! \brief Prints the Twnode */
  void dump() const;
  ///@}

  /** @name Node evaluation accessors */
  ///@{
  /*! \brief Truck's travelTime from previous node to this node.  */
  double travelTime() const { return travelTime_; }
  /*! \brief Truck's arrivalTime to this node.  */
  double arrivalTime() const { return arrivalTime_; }
  /*! \brief Truck's waitTime at this node.  */
  double waitTime() const {return waitTime_;}
  /*! \brief Truck's departureTime from this node.  */
  double departureTime() const { return departureTime_; }
  /*! \brief deltaTime= departureTime(this node) - departureTime(previous). */
  double deltaTime() const { return deltaTime_; }
  ///@}

  /** @name Accumulated evaluation  accessors */
  ///@{
  /*! \brief Truck's total times it has violated time windows.  */
  int  twvTot() const { return twvTot_; }
  /*! \brief Truck's total times it has violated cargo limits.  */
  int  cvTot() const { return cvTot_; }
  /*! \brief Truck's total cargo after the node was served.  */
  double cargo() const { return cargo_; }
  /*! \brief Truck's travel duration up to this node.  */
  double duration() const { return departureTime_; }
  /*! \brief Truck's travel duration up to this node.  */
  double getTotTime() const { return departureTime_;}
  /*! \brief Time spent moving between nodes by the truck */
  double totTravelTime() const { return totTravelTime_; }
  /*! \brief Time spent by the truck waiting for nodes to open */
  double totWaitTime() const { return totWaitTime_; }
  /*! \brief Time spent by the truck servicing the nodes */
  double totServiceTime() const { return totServiceTime_; }
  /*! \brief Truck's total times it has visited a dump in the path.  */
  int dumpVisits() const { return dumpVisits_; }
  ///@}

  /*! \brief True when \barrivalTime + \b deltaTime generates TWV.*/
  double deltaGeneratesTWV(double deltaTime) const;

  /** @name State */
  ///@{
  /*! \brief True when the total count for violations are 0 */
  bool feasable() const { return twvTot_ == 0 && cvTot_ == 0;}
  /*! \brief True doesnt have twc nor cv (including total counts) */
  bool feasable(double cargoLimit) const {
    return feasable() && !has_twv() && !has_cv(cargoLimit);
  }
  /*! \brief True when the Truck at this node doesn not violate time windows */
  bool has_twv() const {
    return lateArrival(arrivalTime_);
  }
  /*! \brief True when the Truck at this node doesn not violate capacity */
  bool has_cv(double cargoLimit) const {
    return cargo_ > cargoLimit || cargo_ < 0;
  }
  ///@}

  /** @name mutators */
  ///@{
  void evaluate(double cargoLimit);
  void evaluate(const Tweval &pred, double cargoLimit);
  #ifdef OSRMCLIENT
  void evaluateOsrm(const Tweval &pred, double cargoLimit);
  #endif
  ///@}

  /** @name Document TWC model functions*/
  ///@{
  /*! \brief returns the Arrival(this) opens(other) arrival time */
  double arrival_i_opens_j(const Tweval &other) const;
  /*! \brief returns the Arrival(this) closes(other) arrival time */
  double arrival_i_closes_j(const Tweval &other) const;
  /*! \brief returns the arriaval(this) arrival(other) arrival time */
  double arrival_i_arrives_j(const Tweval &other, double arrival) const;

  bool isCompatibleIJ(const Tweval &other) const;
  bool isPartiallyCompatibleIJ(const Tweval &other) const;
  bool isFullyCompatibleIJ(const Tweval &other) const;
  bool isNotCompatibleIJ(const Tweval &other) const;
  ///@}



  /*! \brief Construct a default Twnode */
  Tweval();
  /*! \brief Construct from a string line */
  explicit Tweval(std::string line);
  /*! \brief Construct from parameters */
  Tweval(int id, double x, double y, int opens, int closes,
         int serviceTime, int demand, int streetId);

 private:
  /** @name Node evaluation members */
  ///@{
  double travelTime_;     ///< Travel time from last node
  double arrivalTime_;    ///< Arrival time at this node
  double waitTime_;       ///< Wait time at this node is early arrival
  double departureTime_;  ///< Departure time from this node
  double deltaTime_;   ///< Departure time from this node
  ///@}

  /** @name Accumulated evaluation members */
  ///@{
  double cargo_;        ///< Accumulated cargo
  int twvTot_;          ///< Total count of TWV
  int cvTot_;           ///< Total count of CV
  double totWaitTime_;  ///< Accumulated wait time
  double totTravelTime_;   ///< Accumulated travel time
  double totServiceTime_;  ///< Accumulated service time
  int dumpVisits_;      ///< Total count of dump visits
  ///@}
};


#endif  // SRC_BASECLASSES_TWEVAL_H_
