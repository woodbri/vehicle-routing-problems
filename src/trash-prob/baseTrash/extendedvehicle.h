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
#ifndef EXTENDEDVEHICLE_H
#define EXTENDEDVEHICLE_H

#include <limits>
#include <vector>
#include <sstream>


#ifdef DOPLOT
#include "plot.h"
#endif


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "move.h"
#include "basevehicle.h"
//#include "pg_types_vrp.h"


class ExtendedVehicle: public BaseVehicle {
  public:
    //--------------------------------------------------------------------
    // constructors
    //--------------------------------------------------------------------

    ExtendedVehicle(): BaseVehicle() {};

    ExtendedVehicle( int _vid, int _start_id, int _dump_id, int _end_id,
                 double _capacity, double _dumpservicetime, double _starttime,
                 double _endtime, const Bucket &otherlocs )
       :BaseVehicle( _vid, _start_id, _dump_id, _end_id,
                 _capacity, _dumpservicetime, _starttime,
                 _endtime, otherlocs ) {
    }

    ExtendedVehicle( std::string line, const Bucket &otherlocs) 
       : BaseVehicle( std::string line, const Bucket &otherlocs )  {
    }

    const Trashnode& getBackToDepot() const {return endingSite;}

    //double distancetodepot(int i) const { return path[i].distance(getdepot()); };
    //double distancetodump(int i) const { return path[i].distance(getdumpSite()); };

    #ifdef WITHOSRM
    //--------------------------------------------------------------------
    // OSRM stuff
    //--------------------------------------------------------------------
    double getCostOsrm() const;
    double getTotTravelTimeOsrm() const;
    void evaluateOsrm();
    #endif


    //--------------------------------------------------------------------
    // wrappers to twpath code to
    //--------------------------------------------------------------------

    // single path manipulators

    bool push_front( Trashnode node );
    bool remove( int at );
    bool moverange( int rangefrom, int rangeto, int destbefore );
    bool movereverse( int rangefrom, int rangeto, int destbefore );
    bool reverse( int rangefrom, int rangeto );
    bool move( int fromi, int toj );
    bool swap( const int &i, const int &j );

    // multiple path manipulators

    bool swap( ExtendedVehicle &v2, const int &i1, const int &i2 );

    // restore a saved path to undo an operation

    void restorePath( Twpath<Trashnode>
                      oldpath ); //tmp=v; v.dostuff; v=tmp restores as original

    //--------------------------------------------------------------------
    // algorithm specific - intra-route manipulations
    //--------------------------------------------------------------------

    bool doTwoOpt( const int &c1, const int &c2, const int &c3, const int &c4 );
    bool doThreeOpt( const int &c1, const int &c2, const int &c3, const int &c4,
                     const int &c5, const int &c6 );
    bool doOrOpt( const int &c1, const int &c2, const int &c3 );
    bool doNodeMove( const int &i, const int &j );
    bool doNodeSwap( const int &i, const int &j );
    bool doInvertSeq( const int &i, const int &j );

    bool pathOptimize();
    bool pathTwoOpt();
    bool pathThreeOpt();
    bool pathOrOpt();
    bool pathOptMoveNodes();
    bool pathOptExchangeNodes();
    bool pathOptInvertSequence();

    bool findBestFit( const Trashnode &node, int *tpos, double *deltacost );

    //--------------------------------------------------------------------
    // algorithm specific - inter-route manipulations
    //--------------------------------------------------------------------

    bool swap2( ExtendedVehicle &v2, const int &i1, const int &i2, bool force );
    bool swap3( ExtendedVehicle &v2, BaseVehicle &v3, const int &i1, const int &i2,
                const int &i3, bool force );
    bool exchangeSeq( ExtendedVehicle &v2, const int &i1, const int &j1, const int &i2,
                      const int &j2, bool force );
    bool exchangeTails( ExtendedVehicle &v2, const int &i1, const int &i2, bool force );
    bool exchange3( ExtendedVehicle &v2, ExtendedVehicle &v3, const int &cnt, const int &i1,
                    const int &i2, const int &i3, bool force );
    bool relocate( ExtendedVehicle &v2, const int &i1, const int &i2, bool force );
    bool relocateBest( ExtendedVehicle &v2, const int &i1 );
};


#endif

