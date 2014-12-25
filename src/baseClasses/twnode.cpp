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

#include <iostream>
#include <sstream>
#include <string>

#ifdef DOVRPLOG
#include "./logger.h"
#endif

#include "./twnode.h"



/*!
 * \brief Check if a Twnode is valid or not.
 * For Twnode to be valid the following conditions must be true:
 * - id \> -1
 * - tw_open \< tw_close
 * - tw_open \>= 0
 * - serviceTime \>= 0
 */
bool Twnode::isValid() const {
    if ( !Node::isValid() ) return false;

    if ( !(opens_ < closes_
               && opens_ >= 0
               && serviceTime_ >= 0) ) return false;

    if ( type_ == kInvalid )  return false;

    switch  (type_) {
        case kStart:
        case kEnd:
            if (demand_ == 0) return true;
            break;

        case kDump:
            if (demand_ <= 0) return true;
            break;

        case kDelivery:
            if (demand_ < 0) return true;
            break;

        case kPickup:
            if (demand_ > 0) return true;
            break;

        case kLoad:
            if (demand_ >= 0) return true;
            break;

        case kUnknown:
          return true;
          break;

        case kInvalid:
          return false;
          break;
    }
    return false;
}


#ifdef DOVRPLOG
void Twnode::dump() const {
    std::stringstream ss;
    ss.precision(8);
    ss << nid()
       << " = " << id()
       << ",\t\ttype " << type_
       << ",\tx " << x()
       << ",\ty " << y()
       << ",\topen " << opens_
       << ",\tclose " << closes_
       << ",\tdemand " << demand_
       << ",\tserviceT " << serviceTime_
       << ",\t street:" << streetId_
       << ",\t hint:" << hint();
    DLOG(INFO) << ss.str();
}
#endif


void Twnode::set(int nid, int id, double x, double y, double demand,
                  double opens, double closes, double serviceTime) {
    set_nid(nid);
    set_id(id);
    set_x(x);
    set_y(y);
    demand_ = demand;
    opens_ = opens;
    closes_ = closes;
    serviceTime_ = serviceTime;
}

void Twnode::set_demand(int demand) { demand_ = demand; }
void Twnode::set_type(NodeType type) { type_ = type; }
void Twnode::set_opens(int opens) { opens_ = opens; }
void Twnode::set_closes(int closes) { closes_ = closes; }
void Twnode::set_serviceTime(int serviceTime) { serviceTime_ = serviceTime; }
void Twnode::set_streetId(int streetId) { streetId_ = streetId; }


Twnode::Twnode()
      :Node(),
      type_(kUnknown),
      demand_(0),
      opens_(0),
      closes_(0),
      serviceTime_(0),
      streetId_(-1) {
}

/*!
 * \param[in] line A string with space separated values
 * The \c line should be "nid x y tw_open tw_close demand servicetime streetid"
 */
Twnode::Twnode(std::string line)
      :Node(),
      type_(kUnknown),
      demand_(0),
      opens_(0),
      closes_(0),
      serviceTime_(0),
      streetId_(-1) {
  std::istringstream buffer(line);
  int nid;
  double x, y;
  buffer >> nid;
  buffer >> x;
  buffer >> y;
  buffer >> opens_;
  buffer >> closes_;
  buffer >> demand_;
  buffer >> serviceTime_;
  buffer >> streetId_;
  set_id(nid);
  set_nid(nid);
  set_x(x);
  set_y(y);
  type_ = (opens_ < closes_ && opens_ >= 0 && serviceTime_ >= 0)
            ? kUnknown : kInvalid;
}




double Twnode::opens() const {return opens_;}
double Twnode::closes() const {return closes_;}
double Twnode::demand() const {return demand_;}
double Twnode::serviceTime() const { return serviceTime_;}
double Twnode::windowLength() const { return  closes_ - opens_; }
Twnode::NodeType Twnode::type() const {return type_;}
int Twnode::streetId() const {return streetId_;}

bool Twnode::isDepot() const {return type_ == kStart;}
bool Twnode::isStarting() const {return type_ == kStart;}
bool Twnode::isDump() const {return type_ == kDump;}
bool Twnode::isPickup() const {return type_ == kPickup;}
bool Twnode::isEnding() const {return type_ == kEnd;}
bool Twnode::isDelivery() const {return type_ == kDelivery;}
bool Twnode::isLoad() const {return type_ == kLoad;}

bool Twnode::hasDemand() const { return demand_ > 0; }
bool Twnode::hasSupply() const { return demand_ < 0; }
bool Twnode::hasNoGoods() const { return demand_ == 0; }

bool Twnode::earlyArrival(const double arrivalTime) const {
  return arrivalTime < opens_;
}
bool Twnode::lateArrival(const double arrivalTime) const {
  return arrivalTime > closes_;
}
bool Twnode::sameStreet(const Twnode &other) const {
  return streetId_ == other.streetId_;
}
bool Twnode::sameStreet (int streetId) const {return streetId_ == streetId; }

