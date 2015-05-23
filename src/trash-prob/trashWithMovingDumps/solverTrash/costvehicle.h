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
#ifndef COSTVEHICLE_H
#define COSTVEHICLE_H

#include <limits>
#include <vector>
#include <sstream>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOPLOT
#include "plot.h"
#endif

#include "basictypes.h"
#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "move.h"
#include "basevehicle.h"



class CostVehicle: public BaseVehicle
{
public:
const Trashnode& last() const;
double shiftLength() const;
double estimatedZ() const ;
double arrivalEclosesLast(const Trashnode &last) const;
double serviceE() const;




  CostVehicle()
    : ttSC(0.0), ttDC(0.0), ttCD(0.0), ttDE(0.0), ttCC(0.0),
      realttSC(0.0), realttDC(0.0), realttCD(0.0), realttDE(0.0), realttCC(0.0),
      N(0), Nreal(0), minDumpVisits(0), maxDumpVisits(0), realDumpVisits(0),
      Z(0), z1(0), z2(0), realz1(0), realz2(0), n(0), z(0), Zmissing(0), lastn(0),
      totalTime(0.0), realTotalTime(0.0), lastRealTotalTime(0.0),
      forcedWaitTime(0.0), totalWaitTime(0.0), idleTime(0.0),
      realForcedWaitTime(0.0), realtotalWaitTime(0.0), realIdleTime(0.0),
      idleTimeSCDE(0.0), idleTimeSDCDE(0.0),
      realIdleTimeSCDE(0.0), realIdleTimeSDCDE(0.0),
      sumIdle(0.0), penalty(0.0),
      v_cost(0.0), workNotDonePerc(0.0) {
  };


  CostVehicle( std::string line, const Bucket &otherlocs )
    : BaseVehicle( line, otherlocs ),
      ttSC(0.0), ttDC(0.0), ttCD(0.0), ttDE(0.0), ttCC(0.0),
      realttSC(0.0), realttDC(0.0), realttCD(0.0), realttDE(0.0), realttCC(0.0),
      N(0), Nreal(0), minDumpVisits(0), maxDumpVisits(0), realDumpVisits(0),
      Z(0), z1(0), z2(0), realz1(0), realz2(0), n(0), z(0), Zmissing(0), lastn(0),
      totalTime(0.0), realTotalTime(0.0), lastRealTotalTime(0.0),
      forcedWaitTime(0.0), totalWaitTime(0.0), idleTime(0.0),
      realForcedWaitTime(0.0), realtotalWaitTime(0.0), realIdleTime(0.0),
      idleTimeSCDE(0.0), idleTimeSDCDE(0.0),
      realIdleTimeSCDE(0.0), realIdleTimeSDCDE(0.0),
      sumIdle(0.0), penalty(0.0),
      v_cost(0.0), workNotDonePerc(0.0) {

  };

  CostVehicle( int _vid, int _start_id, int _dump_id, int _end_id,
               double _capacity, double _dumpservicetime, double _starttime,
               double _endtime, const Bucket &otherlocs )
    : BaseVehicle( _vid, _start_id, _dump_id, _end_id,
                   _capacity, _dumpservicetime, _starttime,
                   _endtime, otherlocs ),
    ttSC(0.0), ttDC(0.0), ttCD(0.0), ttDE(0.0), ttCC(0.0),
    realttSC(0.0), realttDC(0.0), realttCD(0.0), realttDE(0.0), realttCC(0.0),
    N(0), Nreal(0), minDumpVisits(0), maxDumpVisits(0), realDumpVisits(0),
    Z(0), z1(0), z2(0), realz1(0), realz2(0), n(0), z(0), Zmissing(0), lastn(0),
    totalTime(0.0), realTotalTime(0.0), lastRealTotalTime(0.0),
    forcedWaitTime(0.0), totalWaitTime(0.0), idleTime(0.0),
    realForcedWaitTime(0.0), realtotalWaitTime(0.0), realIdleTime(0.0),
    idleTimeSCDE(0.0), idleTimeSDCDE(0.0),
    realIdleTimeSCDE(0.0), realIdleTimeSDCDE(0.0),
    sumIdle(0.0), penalty(0.0),
    v_cost(0.0), workNotDonePerc(0.0) {
  };


  inline int  realN() const { return ( path.dumpVisits() + 1 ) ;}
  inline double  totalServiceTime() {
    return ( path.totServiceTime() +  dumpSite.serviceTime() +
             endingSite.serviceTime() ) ;
  }

  double getCost() const { return v_cost;};
  double getCost() {
    if (size() > 1) setCost(path.last());
    else setCost(C);

    return v_cost;
  };
  double getCostOsrm() {
    evaluateOsrm();
    if (size() > 1) setCost(path.last());
    else setCost(C);

    return v_cost;
  };

  int getz1() const  {return realz1;};
  int getz2() const {return realz2;};
  int getn() const {return n;};

  void setInitialValues( const Trashnode &node, const Bucket &picks );
  void setCost(const Trashnode &last);
  double getDeltaCost( double deltaTravelTime, int deltan ) ;

#ifdef DOVRPLOG
  void dumpCostValues() const;
#endif
  //for cost function
private:
  Trashnode C;
  double ttSC, ttDC, ttCD, ttDE, ttCC;
  double realttSC, realttDC, realttCD, realttDE, realttCC;
  int N, Nreal, minDumpVisits, maxDumpVisits, realDumpVisits;
  int Z, z1, z2, realz1, realz2, n, z, Zmissing, lastn;
  double totalTime, realTotalTime, lastRealTotalTime;
  double forcedWaitTime, totalWaitTime, idleTime;
  double realForcedWaitTime, realtotalWaitTime, realIdleTime;
  double idleTimeSCDE, idleTimeSDCDE;
  double realIdleTimeSCDE, realIdleTimeSDCDE;
  double sumIdle, penalty;
  double v_cost, workNotDonePerc;
};


#endif

