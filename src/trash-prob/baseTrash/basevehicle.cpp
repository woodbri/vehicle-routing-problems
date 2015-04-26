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
#include <deque>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "stats.h"
#include "timer.h"
#endif

#include "twpath.h"
#include "basevehicle.h"
#ifdef WITHOSRM
#include "vrposrm.h"
#endif
#include "move.h"



void  BaseVehicle::setTravelingTimesOfRoute() const {
    twc->setTravelingTimesOfRoute(path,dumpSite);
}





#ifdef WITHOSRM
double BaseVehicle::getCostOsrm() const
{
  double otime = getTotTravelTimeOsrm();

  // if OSRM failed, return -1.0 to indicate a failure
  if ( otime == -1 ) return otime;

  // WARNING: this is only an approximation because changes at a per
  //          node level need to be evaluated for moving dumps and violations

  return w1 * ( otime + path.getTotWaitTime() + path.getTotServiceTime() ) +
         w2 * endingSite.getcvTot() +
         w3 * endingSite.gettwvTot();
}


double BaseVehicle::getTotTravelTimeOsrm() const
{
  return endingSite.getTotTravelTimeOsrm();
};
#endif

bool BaseVehicle::e_setPath( const Bucket &sol )
{
#ifdef TESTED
  DLOG( INFO ) << "Entering BaseVehicle::e_setPath";
#endif
  assert ( sol.size() );

  if ( not sol.size() ) return false;

  if ( not ( ( sol[0] == path[0] ) and ( sol[sol.size() - 1] == endingSite )
             and ( sol[sol.size() - 2] == dumpSite ) ) )
    return false;

  Bucket tempSol = sol;
  
  tempSol.pop_back();
  tempSol.pop_back();
  tempSol.pop_front();
  
  for (UINT i = 0; i < tempSol.size(); i++) {
      push_back(tempSol[i]);
  }

  assert( ( sol[0] == path[0] ) and ( sol[sol.size() - 1] == endingSite )
          and ( sol[sol.size() - 2] == dumpSite ) );
  return true;
}

bool BaseVehicle::findNearestNodeTo(Bucket &unassigned, POS &pos,
                                    Trashnode &bestNode )
{
#ifdef TESTED
  DLOG( INFO ) << "Entering BaseVehicle::findNearestNodeTo";
#endif
  assert ( unassigned.size() );

  if ( not unassigned.size() ) return false;

  bool flag = false;
  double bestDist = -1;
  double d;

  flag = twc->findNearestNodeUseExistingData(path, unassigned,  pos , bestNode,
         bestDist);

  for ( POS i = 0; i < unassigned.size(); i++ ) {
    if ( twc->isCompatibleIAJ( path[size() - 1]  , unassigned[i], dumpSite ) ) {
      d = unassigned[i].distanceToSegment( path[size() - 1], dumpSite );

      if ( d < bestDist ) {
        bestDist = d;
        bestNode = unassigned[i];
        pos = size();
        flag = true;
      };
    }
  }

  return flag;
}

bool BaseVehicle::findFastestNodeTo(bool first, Bucket &unassigned, POS &pos,
                                    Trashnode &bestNode, double &bestTime) {

#ifdef VRPMAXTRACE
  DLOG( INFO ) << "Entering BaseVehicle::findFastestNodeTo";
#endif
  assert ( unassigned.size() );

  if ( not unassigned.size() ) return false;

  bool flag = false;
  bestTime = VRP_MAX();
  bool oldStateOsrm = osrmi->getUse();
  osrmi->useOsrm(true);
  flag = twc->findFastestNodeTo(first, path, unassigned, dumpSite,
         pos, bestNode, bestTime);

#ifdef VRPMAXTRACE 
DLOG(INFO) << "bestTime: " << bestTime 
           << "\t after position: "  << (pos-1)
           << "\t node:" << bestNode.id() 
           << "\t truck.size():" << path.size();
#endif 
  osrmi->useOsrm(oldStateOsrm);
  return flag;
}



#ifdef DOVRPLOG
void BaseVehicle::dump() const {
  DLOG( INFO ) << "---------- BaseVehicle ---------------";
  DLOG( INFO ) << "maxcapacity: " << getmaxcapacity();
  DLOG( INFO ) << "cargo: " << getCargo();
  DLOG( INFO ) << "twvTot: " << twvTot();
  DLOG( INFO ) << "cvTot: " << cvTot();
  DLOG( INFO ) << "path nodes: -----------------";
  path.dump();
}


void BaseVehicle::dumppath() const
{
  path.dump();
}

void BaseVehicle::dump( const std::string &title ) const
{
  path.dump( title );
}


void BaseVehicle::dumpeval() const
{
  DLOG( INFO ) << "\nStarting site:";
  path[0].dumpeval(maxcapacity);

  for ( POS i = 1; i < path.size(); i++ ) {
    DLOG( INFO ) << "path stop #:" << i;
    path[i].dumpeval(maxcapacity);
  }

  DLOG( INFO ) << "Dump site:";
  dumpSite.dumpeval(maxcapacity);
  DLOG( INFO ) << "Ending site :";
  endingSite.dumpeval(maxcapacity);
  DLOG( INFO )  << "TOTAL COST=" << cost;
}


void BaseVehicle::smalldump() const
{
  DLOG( INFO ) << "Dump site:";
  dumpSite.dumpeval(maxcapacity);
  DLOG( INFO ) << "Ending site:";
  endingSite.dumpeval(maxcapacity);
  DLOG( INFO )  << "TOTAL COST=" << cost;
}

void BaseVehicle::tau() const
{
  std::stringstream ss;
  ss << " ";

  for (POS i = 0; i < path.size(); i++) {
    ss << id(i) << " ";
    if (path[i].isDump()) ss << "\n" << id(i) << " ";
  }

  ss << dumpSite.id() << " ";
  ss << endingSite.id();
  DLOG( INFO ) << ss.str();
}
#endif

std::deque<int> BaseVehicle::getpath() const
{
  std::deque<int> p;
  p = path.getpath();
  p.push_front( getDepot().nid() );
  p.push_back( getDumpSite().nid() );
  p.push_back( getDepot().nid() );
  return p;
}

bool BaseVehicle::e_adjustDumpsToNoCV(int currentPos) {
  path.e_adjustDumpsToNoCV(currentPos, dumpSite, maxcapacity);
}


bool BaseVehicle::push_back( Trashnode node )
{
  if (not  path.e_push_back( node, getmaxcapacity()))
    return false;

  evalLast();
  return true;
}

#if 0
bool BaseVehicle::push_front( Trashnode node )
{
  // position 0 is the depot we can not put a node before that
  return insert( node, 1 );
}

#endif

bool BaseVehicle::e_insert( Trashnode node, int at )
{
  if (not path.e_insert( node, at, getmaxcapacity()))
    return false;

  evalLast();
  return true;
}

 void BaseVehicle::e_swap(int i,int j){  
        if (i==j) return; //nothing to swap
        path.e_swap(i,j,maxcapacity);
        evalLast();
 }


bool BaseVehicle::e_remove( int at )
{
  bool ret = path.e_remove(at, getmaxcapacity());
  evalLast();
  return ret;
}


#if 0
bool BaseVehicle::moverange( int rangefrom, int rangeto, int destbefore )
{
  E_Ret ret = path.e_move( rangefrom, rangeto, destbefore, getmaxcapacity() );

  if ( ret == OK ) evalLast();
  else if ( ret == INVALID ) return false;

  return true;
}


bool BaseVehicle::movereverse( int rangefrom, int rangeto, int destbefore )
{
  E_Ret ret = path.e_movereverse( rangefrom, rangeto, destbefore,
                                  getmaxcapacity() );

  if ( ret == OK ) evalLast();
  else if ( ret == INVALID ) return false;

  return true;
}


bool BaseVehicle::reverse( int rangefrom, int rangeto )
{
  E_Ret ret = path.e_reverse( rangefrom, rangeto, getmaxcapacity() );

  if ( ret == OK ) evalLast();
  else if ( ret == INVALID ) return false;

  return true;
}


bool BaseVehicle::move( int fromi, int toj )
{
  E_Ret ret = path.e_move( fromi, toj, getmaxcapacity() );

  if ( ret == OK ) evalLast();
  else if ( ret == INVALID ) return false;

  return true;
}


bool BaseVehicle::swap( const int &i, const int &j )
{
  E_Ret ret = path.e_swap( i, j, getmaxcapacity() );

  if ( ret == OK ) evalLast();
  else if ( ret == INVALID ) return false;

  return true;
}


bool BaseVehicle::swap( BaseVehicle &v2, const int &i1, const int &i2 )
{
  E_Ret ret = path.e_swap( i1, getmaxcapacity(),
                           v2.getvpath(), i2, v2.getmaxcapacity() );

  if ( ret == OK ) {
    evalLast();
    v2.evalLast();
  } else if ( ret == INVALID ) return false;

  return true;
}


void BaseVehicle::restorePath( Twpath<Trashnode> oldpath )
{
  path = oldpath;
  evalLast();
}
#endif

void BaseVehicle::evalLast()
{
  Trashnode last = path[path.size() - 1];
  dumpSite.set_demand( -last.cargo() );
  dumpSite.evaluate( last, getmaxcapacity() );
  endingSite.evaluate( dumpSite, getmaxcapacity() );

  // if a vehcile has no containers to pickup it is empty
  // and empty trucks do not leave leave the depot
  // so cost for path.size()-1 == 0 is 0
  if ( path.size() - 1 == 0 )
    cost = 0.0;
  else
    cost = w1 * endingSite.getTotTime() +
           w2 * endingSite.cvTot() +
           w3 * endingSite.twvTot();
}

void BaseVehicle::evaluate() {
  setTravelingTimesOfRoute();  // uses osrm
  path.evaluate(0, getmaxcapacity());
  evalLast();
}


#ifdef OSRMCLIENT
void BaseVehicle::evaluateOsrm() {
  if (!osrmi->getConnection()) {
    DLOG(INFO)<<"OSRM connection not found: using normal evaluation";
    evaluate();
    return;
  };
  bool oldUse=osrmi->getUse();
  osrmi->useOsrm(true);
  path.evaluateOsrm(getmaxcapacity());

  Trashnode last = path[path.size() - 1];
  dumpSite.evaluateOsrm(last, getmaxcapacity());
  endingSite.evaluate(dumpSite, getmaxcapacity());
  osrmi->clear();
  osrmi->useOsrm(oldUse);
};
#endif




#ifdef DOPLOT
/**************************************PLOT************************************/
void BaseVehicle::plot( std::string file, std::string title,
                        int carnumber ) const
{
  //    DLOG(INFO) << "USING VEHICLE PLOT";
  Twpath<Trashnode> trace = path;
  trace.push_back( dumpSite );
  trace.push_back( endingSite );
  trace.dumpid( "Path" );
  trace.pop_front();
  trace.pop_back();
  trace.pop_back();
  /** cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber */
  std::stringstream convert;
  convert << carnumber;
  std::string carnum = convert.str();
  std::string extra = "vehicle" + carnum ;

  Plot<Trashnode> graph( trace );
  graph.setFile( CONFIG->getString( "plotDir" ) + file + extra + ".png" );
  graph.setTitle( title + extra );
  graph.drawInit();

  for ( int i = 0; i < trace.size(); i++ ) {
    if ( trace[i].isPickup() )  {
      graph.drawPoint( trace[i], 0x0000ff, 9, true );
    } else if ( trace[i].isDepot() ) {
      graph.drawPoint( trace[i], 0x00ff00, 5, true );
    } else  {
      graph.drawPoint( trace[i], 0xff0000, 7, true );
    }
  }

  plot( graph, carnumber );
  graph.save();
}

void BaseVehicle::plot( Plot<Trashnode> graph, int carnumber )const
{
  //DLOG(INFO) << "USING VEHICLE PLOT  1";
  Twpath<Trashnode> trace = path;
  trace.push_back( dumpSite );
  trace.push_back( endingSite );
  graph.drawPath( trace, graph.makeColor( carnumber * 10 ), 1, true );
}
#endif



BaseVehicle::BaseVehicle( int _vid, int _start_id, int _dump_id, int _end_id,
                          double _capacity, double _dumpservicetime, double _starttime,
                          double _endtime, const Bucket &otherlocs )
  : vid(-1),
    startTime(0), endTime(0),
    maxcapacity(0), cost(0),
    w1(1.0), w2(1), w3(1.0)
{

  assert( otherlocs.size() );

  int depotId, dumpId, endingId;
  double dumpServiceTime;

  vid             = _vid;
  depotId         = _start_id;
  dumpId          = _dump_id;
  endingId        = _end_id;
  maxcapacity     = _capacity;
  dumpServiceTime = _dumpservicetime;
  startTime       = _starttime;
  endTime         = _endtime;

  if ( depotId >= 0 and dumpId >= 0 and endingId >= 0 and
       startTime >= 0 and startTime <= endTime and maxcapacity > 0 and
       dumpServiceTime >= 0 and vid >= 0 and otherlocs.hasId( depotId ) and
       otherlocs.hasId( dumpId ) and otherlocs.hasId( endingId ) ) {

    endingSite = otherlocs[otherlocs.posFromId( endingId )];

    if ( endingSite.closes() > endTime )
      endingSite.set_closes( endTime );

    dumpSite = otherlocs[otherlocs.posFromId( dumpId )];
    dumpSite.set_serviceTime( dumpServiceTime );
    depot = otherlocs[otherlocs.posFromId( depotId )];

    if ( depot.opens() < startTime )
      depot.set_opens( startTime );

    depot.set_type( Twnode::kStart );
    depot.set_demand( 0 );
    dumpSite.set_type( Twnode::kDump );
    endingSite.set_type( Twnode::kEnd );
    push_back( depot );
    evalLast();
  } else
    vid = -1; //truck is rejected
}

BaseVehicle::BaseVehicle(const std::string &line, const Bucket &otherlocs)
  : vid(-1),
    startTime(0), endTime(0),
    maxcapacity(0), cost(0),
    w1(1.0), w2(1.0), w3(1.0)
{
  // TESTED on running program
  assert( otherlocs.size() );
  std::istringstream buffer( line );
  int depotId, dumpId, endingId;
  double dumpServiceTime;

  buffer >> vid;
  buffer >> depotId;
  buffer >> dumpId;
  buffer >> endingId;
  buffer >> dumpServiceTime;
  buffer >> maxcapacity;
  buffer >> startTime;
  buffer >> endTime;

  if ( depotId >= 0 and dumpId >= 0 and endingId >= 0 and startTime >= 0
       and startTime <= endTime and maxcapacity > 0 and dumpServiceTime >= 0
       and vid >= 0
       and otherlocs.hasId( depotId ) and otherlocs.hasId( dumpId )
       and otherlocs.hasId( endingId ) ) {

    endingSite = otherlocs[otherlocs.posFromId( endingId )];

    if ( endingSite.closes() > endTime ) endingSite.set_closes( endTime );

    dumpSite = otherlocs[otherlocs.posFromId( dumpId )];
    dumpSite.set_serviceTime( dumpServiceTime );
    depot = otherlocs[otherlocs.posFromId( depotId )];

    if ( depot.opens() < startTime ) depot.set_opens( startTime );

    depot.set_type( Twnode::kStart );
    depot.set_demand( 0 );
    dumpSite.set_type( Twnode::kDump );
    endingSite.set_type( Twnode::kEnd );
    push_back( depot );
    evalLast();
    //dumpeval();

  } else vid = -1; //truck is rejected
}

BaseVehicle::BaseVehicle()
  : vid(-1),
    startTime(0), endTime(0),
    maxcapacity(0), cost(0),
    w1(1.0), w2(1.0), w3(1.0)
{
  path.clear();
};

