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
#ifndef SRC_BASECLASSES_TWNODE_H_
#define SRC_BASECLASSES_TWNODE_H_

#include <string>

#include "./node.h"

/*! \class Twnode
 * \brief Extends the \ref Node class to create a Node with time window attributes.
 *
 * A Time Window node is a Node with addition attributes and methods to
 * to support Time Windows and to model a more complex Node need in many
 * vehicle routing problems.
 *
 * Most application specific code will extend this class and define the specific
 * values and requirements for \c type and \c streetid.
 *
 * Currently the trash collection problem is using node type values of:
 * - -1: Invalid
 * - 0: Depot or Start location
 * - 1: Dump site
 * - 2: Pickup location
 * - 3: End site
 * Currently the pick & delivery problem is using node type values of:
 * - -1: Invalid
 * - 0: Depot or Start location also as ending site
 * - was using 1 as delivery, but the trash collection problem is more urgent
 * \todo use something else as deliver TODO
 * - 2: Pickup location

 * It should be noted the times are normally defined by some amount of elapsed
 * time from some problem start time of 0.0. The actual problem will specify
 * what the time units are, like seconds, minutes, hours etc.
 */
class Twnode: public Node {
 public:
  typedef enum {
    kInvalid = -2,  ///< an invalid or undefined move
    kUnknown = -1,  ///< an invalid or undefined move
    kStart = 0,     ///< starting site
    kDump = 1,      ///< dump site, empties truck
    kPickup = 2,    ///< pickup site
    kEnd = 3,       ///< ending site
    kDelivery = 4,   ///< delivery site
    kLoad = 5       ///< load site, fills the truck
  } NodeType;


  /** @name accessors */
  ///@{
  /*! \brief Get the opening time.  */
  double opens() const;

  /*! \brief Get the closing time.  */
  double closes() const;

  /*! \brief Get the demand associated with this node.  */
  double demand() const;

  /*!  * \brief Get the service time for this node.  */
  double serviceTime() const;

  /*!  \brief Get the length of time between the opening and closeing.  */
  double windowLength() const;

  /*!  \brief Get the type of node.  */
  NodeType type() const;

  /*! \brief Get the street id or -1 when it is not defined.  */
  int streetId() const;
  ///@}


  /** @name type of node*/
  ///@{
  bool isDepot() const;
  bool isStarting() const;
  bool isDump() const;
  bool isPickup() const;
  bool isEnding() const;
  bool isDelivery() const;
  bool isLoad() const;
  ///@}


  /*!  * \brief Print the contents of a Twnode object.  */
  void dump() const;

  // doc in cpp
  bool isValid() const;
  bool isvalid() const;

  /** @name demand of node*/
  ///@{
  /*! \brief True when the node's demand is positive */
  bool hasDemand() const;

  /*! \brief True when the node's demand is negative */
  bool hasSupply() const;

  /*! \brief True when the node's demand is 0. */
  bool hasNoGoods() const;
  ///@}


  /*! \brief True when \b \c arrivalTime is before it \b opens */
  bool earlyArrival(const double arrivalTime) const;

  /*! \brief True when \b \c arrivalTime is after it \b closes */
  bool lateArrival(const double arrivalTime) const;


  /** @name sameStreet
    \return True when the street ids match */
  ///@{
  /*! \param[in] other node. */
  bool sameStreet(const Twnode &other) const;
  /*! \param[in] streetId */
  bool sameStreet(int streetId) const;
  ///@}


  /** @name mutators */
  ///@{
  /*!  \brief Set the attributes of a Twnode object.

   \param[in] nid Value for internal node id
   \param[in] id Value for user node id
   \param[in] x Value of the x or longitude coordinate
   \param[in] y Value of the y or latitude coordinate
   \param[in] demand Value of the demand for this node
   \param[in] opens The earliest possible arrival time
   \param[in] closes The latest possible arrivial time
   \param[in] serviceTime The length of time to service this node
   \param[in] setreetId of the node
  */
  void set(int nid, int id, double x, double y, double demand,
           double opens, double closes, double serviceTime
           int streetId);
  void set(int nid, int id, double x, double y, double demand,
           double opens, double closes, double serviceTime);

  /*!  * \brief Set the demand for this node.  */
  void set_demand(int demand);

  /*!  * \brief Set the \b \c type of this node.  */
  void set_type(NodeType type);

  /*!  * \brief Set the time that the node \b \c opens.  */
  void set_opens(int opens);

  /*!  * \brief Set the time that node \b \c closes.  */
  void set_closes(int closes);

  /*!  * \brief Set the \b \c serviceTime for this node.  */
  void set_serviceTime(int serviceTime);

  /*!  * \brief Set the \b \c streetId for this node.  */
  void set_streetId(int streetId);
  ///@}

  /*! \brief Construct an undefined Twnode object.  */
  Twnode();

  /*! \brief Construct from a string */
  explicit Twnode(std::string line);

 private:
  NodeType type_;       ///< Defines what type of Twnode
  double demand_;       ///< The demand for the Node
  double opens_;        ///< opening time of the node
  double closes_;       ///< closing time of the node
  double serviceTime_;  ///< time it takes to be served
  int streetId_;        ///< The street id of the node
};

#endif  // SRC_BASECLASSES_TWNODE_H_
