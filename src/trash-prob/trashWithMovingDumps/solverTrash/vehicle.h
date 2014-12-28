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
#ifndef VEHICLE_H
#define VEHICLE_H

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
#include "costvehicle.h"



class Vehicle: public CostVehicle
{

protected:
  /*    typedef  TwBucket<Trashnode> Bucket;
      typedef  unsigned long int UID ;
      typedef  unsigned long int POS ;
      inline double _MAX() const { return ( std::numeric_limits<double>::max() ); };
      inline double _MIN() const { return ( - std::numeric_limits<double>::max() ); };
  */
  typedef std::set<Move, Move::compMove> Moves;

public:
  // TODO LIST
  // insertion will not be performed
  //  return false if TV or CV is generated
  bool eval_insertSteadyDumps( const Trashnode &node, POS at ) const;

  bool e_insertIntoFeasableTruck( const Trashnode &node, POS pos );
  // insertion will be performed and return false if TV or CV is generated
  bool e_insertMoveDumps( const Trashnode &node, POS at );
  bool e_insertSteadyDumps( const Trashnode &node, POS at );
  bool e_insert( const Trashnode &node, POS at ) { return  e_insertMoveDumps( node, at ); };


  // Very TIGHT insertion
  // insertion will not be performed if
  //      TV and CV are  generated
  //  true- insertion was done
  //  false- not inserted
  bool e_insertMoveDumpsTight( const Trashnode &node, POS at );
  bool e_insertSteadyDumpsTight( const Trashnode &node, POS at );
  bool e_insertTight( const Trashnode &node, POS at ) { return  e_insertMoveDumpsTight( node, at ); };
  // END TODO LIST

  bool applyMoveINSerasePart( UID nodeNid, POS pos );
  bool applyMoveINSinsertPart( const Trashnode &node, POS pos );
  bool applyMoveInterSw( Vehicle &otherTruck, POS truckPos, POS otherTruckPos );
  bool applyMoveIntraSw( POS fromPos, POS withPos );
  bool e_makeFeasable( POS currentPos );
  long int eval_insertMoveDumps( const Trashnode &node, std::deque<Move> &moves,
                                 POS fromTruck, POS formPos, POS toTruck, double savings, double factor ) const;
  long int eval_intraSwapMoveDumps( std::deque<Move> &moves, POS  truckPos,
                                    POS fromPos ) const ;
  bool e_insertDumpInPath( const Trashnode &going );
  bool deltaTimeGeneratesTV( const Trashnode &dump, const Trashnode &node ) const;
  bool deltaCargoGeneratesCV( const Trashnode &node, POS pos ) const;
  bool deltaCargoGeneratesCV_AUTO( const Trashnode &node, POS pos ) const;
  bool deltaTimeGeneratesTV( const Trashnode &node, POS pos ) const;
  bool deltaTimeGeneratesTV_AUTO( const Trashnode &node, POS pos ) const;
  //--------------------------------------------------------------------
  // constructors
  //--------------------------------------------------------------------

  Vehicle() {
  };


  Vehicle( std::string line, const Bucket &otherlocs )
    : CostVehicle( line, otherlocs )   { }


  Vehicle( int _vid, int _start_id, int _dump_id, int _end_id,
           int _capacity, int _dumpservicetime, int _starttime,
           int _endtime, const Bucket &otherlocs )
    : CostVehicle( _vid, _start_id, _dump_id, _end_id,
                   _capacity, _dumpservicetime, _starttime,
                   _endtime, otherlocs ) {};


  double timePCN( POS from, POS middle, POS to ) const;
  double timePCN( POS from, Trashnode &middle ) const;

  long int eval_intraSwapMoveDumps( Moves &moves, POS  truckPos) const;
  long int eval_interSwapMoveDumps( Moves &moves, const Vehicle &otherTruck,
                                    POS  truckPos, POS  otherTruckPos,
                                    double factor   ) const;
  long int eval_interSwapMoveDumps( Moves &moves, const Vehicle &otherTruck,
                                    POS  truckPos, POS  otherTruckPos,
                                    POS fromPos, POS toPos   ) const;
  long int eval_insertMoveDumps( const Trashnode &node, Moves &moves,
                                 POS fromTruck, POS formPos, POS toTruck,
                                 double savings ) const;
  bool eval_erase( POS at, double &savings ) const;
};


#endif

