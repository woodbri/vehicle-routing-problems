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
#include <string>
#include <iostream>
#include <sstream>

#ifdef DOVRPLOG
#include "./logger.h"
#endif


#include "./tweval.h"


/*!
 * \brief Evaluate a node at the start of a path. (has to be a depot)
 * \param[in] cargoLimit The cargo limit for the vehicle
 */
void Tweval::evaluate(double cargoLimit) {
    assert(isStarting());

    cargo_ = demand();
    travelTime_ = 0;
    arrivalTime_ = opens();
    totTravelTime_ = 0;
    totWaitTime_ = 0;
    totServiceTime_ = serviceTime();
    departureTime_ = arrivalTime_ + serviceTime();
    twvTot_ = cvTot_ = 0;
    dumpVisits_ = 0;
    cvTot_ = (cargo_ > cargoLimit ? 1 : 0) == 1;
    deltaTime_ = 0;
}

/*!
 * \brief Evaluate a node in the path and update path totals from predecessor
 *
 * A path is a sequence of nodes and we maintain path statistics by keeping
 * a running sum of path related variable on each node in the path. So the
 * last node in the path will always reflect the total path. This method
 * updates those path variables based on the predecssor node.
 *
 * \param[in] pred The node preceeding this node in the path.
 * \param[in] cargoLimit The cargo limit for this vehicle.
 */
void Tweval::evaluate(const Tweval &pred, double cargoLimit) {
    travelTime_    = twc->TravelTime(pred.nid(), nid());
    totTravelTime_ = pred.totTravelTime_ + travelTime_;
    arrivalTime_   = pred.departureTime_ + travelTime_;
    waitTime_      = earlyArrival(arrivalTime_) ? opens() - arrivalTime_ : 0;
    totWaitTime_   = pred.totWaitTime_ + waitTime_;
    totServiceTime_ = pred.totServiceTime_ + serviceTime();
    departureTime_  = arrivalTime_ + waitTime_ + serviceTime();

    if ( isDump()  and pred.cargo_ >= 0 )
      set_demand(-pred.cargo_);     // type 1 empties the truck (aka dumpSite)

    dumpVisits_ = isDump()? pred.dumpVisits_ + 1 :  pred.dumpVisits_;
    cargo_ = pred.cargo_ + demand();            // loading truck demand>0 or unloading demand<0

    twvTot_ = has_twv()? pred.twvTot_ + 1 : pred.twvTot_;
    cvTot_ =  has_cv(cargoLimit)? pred.cvTot_ + 1 : pred.cvTot_;
    deltaTime_ = departureTime_ - pred.departureTime_;
}


#ifdef DOVRPLOG
/*! \brief Print the Twnode */
void Tweval::dump() const {
    Twnode::dump();
}

/*! \brief Print the Tweval attributes for the node.  */
void Tweval::dumpeval(double cargoLimit) const  {
    DLOG(INFO) << "twv=" << has_twv()
                 << ", cv=" << has_cv(cargoLimit)
                 << ", twvTot=" << twvTot_
                 << ", cvTot=" << cvTot_
                 << ", cargo=" << cargo_
                 << ", travel Time=" << travelTime_
                 << ", arrival Time=" << arrivalTime_
                 << ", wait Time=" << waitTime_
                 << ", service Time=" << serviceTime()
                 << ", departure Time=" << departureTime_;
}
#endif


/*! \brief Construct a default Twnode */
Tweval::Tweval()
       :Twnode(),
        travelTime_(0), arrivalTime_(0), waitTime_(0), departureTime_(0), deltaTime_(0),
        cargo_(0), twvTot_(0), cvTot_(0),
        totWaitTime_(0), totTravelTime_(0), totServiceTime_(0), dumpVisits_(0) {
}


/*! \brief Construct a Tweval node from a text string, typically read from a file.  */
Tweval::Tweval(std::string line)
       :Twnode(line),
        travelTime_(0), arrivalTime_(0), waitTime_(0), departureTime_(0), deltaTime_(0),
        cargo_(0), twvTot_(0), cvTot_(0),
        totWaitTime_(0), totTravelTime_(0), totServiceTime_(0), dumpVisits_(0) {
}

/*! \brief Construct a Tweval node from arguments

 * \param[in] id The User node id
 * \param[in] x The X or longitude coordinate for its location
 * \param[in] y The Y or latitude coordinate for its location
 * \param[in] opens The earliest arrival time (TW open)
 * \param[in] closes The latest arrival time (TW close)
 * \param[in] serviceTime The service time
 * \param[in] demand The demand in units of vehicle capacity
 * \param[in] streetId The street id this node is located
 */
Tweval::Tweval(int id, double x, double y, int opens, int closes,
               int serviceTime, int demand, int streetId)
       :Twnode(),
        travelTime_(0), arrivalTime_(0), waitTime_(0), departureTime_(0), deltaTime_(0),
        cargo_(0), twvTot_(0), cvTot_(0),
        totWaitTime_(0), totTravelTime_(0), totServiceTime_(0), dumpVisits_(0) {
    set(id, id, x, y, demand, opens, closes, serviceTime);
    set_streetId(streetId);
}



/*!
 * \brief Test if adding some delta time to its arraive causes it to violate its time window.
 */
double Tweval::deltaGeneratesTWV(double deltaTime) const {
    return ( arrivalTime_ + deltaTime > closes() );
}


#if 0
#ifdef WITHOSRM
/*!
 * \brief Return the location as an OSRM loc string
 */
std::string Tweval::getLoc() const {
    std::ostringstream buf;
    buf << "&loc=" << y << "," << x;
    return buf.str();
}

/*!
 * \brief Construct the OSRM Url needed to get the route from start to this node.
 * \param[in] osrmBaseUrl This is the base URL for the OSRM server to use.
 * \return A std::string as the full URL for the path to this node.
 */
std::string Tweval::getOsrmUrl(const std::string osrmBaseUrl) const {
    return osrmBaseUrl + "viaroute?z=18&instructions=false&alt=false" + osrmUrlLocs;
}

/*!
 * \brief Evaluate OSRM attributes for the first node in a path
 */
void Tweval::evaluateOsrm() {
    totTravelTimeOsrm = 0.0;
    osrmUrlLocs = getLoc();
}

/*!
 * \brief Evaluate OSRM attributes for all the successor nodes in the path
 *
 * Construct the OSRM URL based on the previous path components. Call OSRM and
 * get the travel time for the path from start to this point and save it on
 * this node. If there is an error this and all soccessor nodes will be set
 * in an error state, IE: getTotTravelTimeOsrm() will return -1.0.
 *
 * \param[in] osrmBaseUrl The OSRm base URl passed in from application.
 * \param[in] pred The previous node in the path.
 */
void Tweval::evaluateOsrm(const Tweval &pred,
                          const std::string &osrmBaseUrl) {
    // if there was a previous error in the path computing OSRM time
    // then we have already failed so just fail here and return
    double ttimePred = pred.getTotTravelTimeOsrm();

    if ( ttimePred == -1 ) {
        totTravelTimeOsrm = -1.0;
        return;
    }

    // append this nodes loc to the previous ones in the path
    osrmUrlLocs = pred.getOsrmUrlLocs() + getLoc();

    // construct the complete URL
    std::string url = getOsrmUrl(osrmBaseUrl);
    VrpOSRM osrm;

    // if we fail to get the OSRM time then set the error indicator and return
    if ( osrm.callOSRM(url) ) {
        totTravelTimeOsrm = -1.0;
        return;
    }

    // if we fail to get the OSRM time then set the error indicator and return
    double ttime;

    if ( osrm.getTravelTime(ttime) ) {
        totTravelTimeOsrm = -1.0;
        return;
    }

    // set the total travel time for OSRM to this point in the path
    totTravelTimeOsrm = ttime;
}

#endif
#endif  // 0
