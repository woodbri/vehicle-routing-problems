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
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#include "prob_trash.h"

// Class functions
#if 0
bool Prob_trash::checkIntegrity() const
{
  bool flag = true;
  int nodesCant = datanodes.size();

  if ( datanodes.empty() ) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "Nodes is empty";
#endif
    flag = false;
  }

#ifdef DOVRPLOG
  else DLOG( INFO ) << "# of Nodes:" << nodesCant;

#endif

  for ( int i = 1; i < nodesCant - 1; i++ ) {
    flag = flag and datanodes[i].isValid();
  }
}
#endif

// DUMPS ********************************************
#ifdef DOVRPLOG
void Prob_trash::nodesdump()
{
  DLOG( INFO ) << "---- Nodes  --------------";

  for ( UINT i = 0; i < datanodes.size(); i++ )
    datanodes[i].dump();
}


#if 0
void Prob_trash::nodesdumpeval()
{
  DLOG( INFO ) << "---- Nodes  Evaluation--------------";

  for ( UINT i = 0; i < datanodes.size(); i++ )
    datanodes[i].dumpeval();
}


void Prob_trash::dump()
{
  DLOG( INFO ) << "---- Problem -------------";
  nodesdump();
  DLOG( INFO ) << "INITIAL EVALUATION";
  nodesdumpeval();

}
#endif

void Prob_trash::dumpdataNodes() const
{
  DLOG( INFO ) << "--------- Nodes ------------";

  for ( UINT i = 0; i < datanodes.size(); i++ )
    datanodes[i].dump();
}



void Prob_trash::dumpDepots() const
{
  DLOG( INFO ) << "--------- Depots ------------";
  depots.dump( "Depots" );

  for ( UINT i = 0; i < depots.size(); i++ )
    depots[i].dump();
}



void Prob_trash::dumpDumps() const
{
  DLOG( INFO ) << "--------- Dumps ------------";
  dumps.dump( "Dumps" );

  for ( UINT i = 0; i < dumps.size(); i++ )
    dumps[i].dump();
}


void Prob_trash::dumpPickups() const
{
  DLOG( INFO ) << "--------- Pickups ------------";
  pickups.dump( "pickups" );

  for ( UINT i = 0; i < pickups.size(); i++ )
    pickups[i].dump();
}
#endif

// PLOTS  ********************************************
#ifdef DOPLOTS
void Prob_trash::plot( Plot<Trashnode> &graph )
{
  for ( int i = 0; i < datanodes.size(); i++ ) {
    if ( datanodes[i].isPickup() )  {
      graph.drawPoint( datanodes[i], 0x0000ff, 9, true );
    } else if ( datanodes[i].isDump() ) {
      graph.drawPoint( datanodes[i], 0x00ff00, 5, true );
    } else  {
      graph.drawPoint( datanodes[i], 0xff0000, 7, true );
    }
  }
};
#endif



Prob_trash::Prob_trash( const char *infile )
{
#ifdef VRPMINTRACE
  DLOG( INFO ) << "---- char * Constructor --------------";
#endif
  std::string file = infile;
  loadProblem( file );
}

Prob_trash::Prob_trash( const std::string &infile )
{
#ifdef VRPMINTRACE
  DLOG( INFO ) << "Prob_trash---- string Constructor --------------";
#endif
  loadProblem( infile );
}


void Prob_trash::loadProblem( const std::string &infile )
{
  datafile = infile;
  Bucket nodes;
  Bucket intersection;
#ifdef VRPMINTRACE
  DLOG( INFO ) << "Prob_trash LoadProblem --------------" << datafile <<
               "--------";
#endif


  // read the nodes
  //int cnt = 0;
  //int nid = 0;
  int id = 0;

  twc->emptiedTruck = false;

  load_pickups( datafile + ".containers.txt" );
  load_otherlocs( datafile + ".otherlocs.txt" );

  intersection = otherlocs * pickups;
  invalid = invalid + intersection;
  pickups = pickups - intersection;
  nodes = nodes - intersection;

#ifdef VRPMINTRACE
  invalid.dump( "invalid" );
#endif


  nodes = pickups + otherlocs;
  nodes.push_back( C );

  for ( UINT i = 0; i < nodes.size(); i++ ) {
    nodes[i].set_nid( i );
    id = nodes[i].id();

    if ( pickups.hasId( id ) )
      pickups[ pickups.posFromId( id ) ].set_nid( i );
    else if ( otherlocs.hasId( id ) )
      otherlocs[ otherlocs.posFromId( id ) ].set_nid( i );
  };

  C = nodes.back();
  assert( pickups.size() );
  assert( otherlocs.size() );

  datanodes = nodes;



  twc->loadAndProcess_distance( datafile + ".dmatrix-time.txt", datanodes,
                                invalid );
  load_trucks( datafile + ".vehicles.txt" );

#ifdef OSRMCLIENT
#ifdef VRPMINTRACE
  DLOG(INFO) << "Setting hints";
#endif
  twc->setHints( dumps );
#ifdef VRPMINTRACE
  DLOG(INFO) << "    hints for dumps done";
#endif
  twc->setHints( nodes );
#ifdef VRPMINTRACE
  DLOG(INFO) << "    hints for nodes done";
#endif
  twc->setHints( depots );
#ifdef VRPMINTRACE
  DLOG(INFO) << "    hints for depots done";
#endif
  twc->setHints( pickups );
#ifdef VRPMINTRACE
  DLOG(INFO) << "    hints for pickups done";
#endif
  twc->setHints( endings );
#ifdef VRPMINTRACE
  DLOG(INFO) << "    hints for endings done";
#endif
  twc->settCC( C, pickups );
#ifdef VRPMINTRACE
  DLOG(INFO) << "    settCC for pickups done";
#endif
#endif  // OSRMCLIENT

  assert( trucks.size() and depots.size() and dumps.size() and endings.size() );

  for ( UINT i = 0; i < trucks.size(); i++ ) {
    trucks[i].setInitialValues( C, pickups );
  }
#ifdef VRPMINTRACE
  DLOG(INFO) << "trucks[i].setInitialValues( C, pickups ) done";
#endif

#ifdef VRPMAXTRACE
  C.dump();
  nodes.dump( "nodes" );
  dumps.dump( "dumps" );
  depots.dump( "depots" );
  pickups.dump( "pickups" );
  endings.dump( "endings" );
  datanodes.dump( "datanodes" );
  invalid.dump( "invalid" );
  DLOG( INFO ) << "TRUCKS";

  for ( int i = 0; i < trucks.size(); i++ ) trucks[i].tau();

#ifdef VRPMINTRACE
  DLOG( INFO ) << "INVALID TRUCKS";
#endif
  if (invalidTrucks.size()==0) DLOG( INFO ) << " NONE\n";
  for ( int i = 0; i < invalidTrucks.size(); i++ ) invalidTrucks[i].tau();

  //twc->dump();
#endif
#ifdef VRPMINTRACE
  DLOG( INFO ) << "-------- Leaving Prob_trash::LoadProblem --------------";
#endif
}



void Prob_trash::load_trucks( std::string infile )
{
  assert ( otherlocs.size() );
  std::ifstream in( infile.c_str() );
  std::string line;
#ifdef VRPMINTRACE
  DLOG( INFO ) << "Prob_trash:LoadTrucks" << infile;
#endif

  trucks.clear();

  while ( getline( in, line ) ) {

    if ( line[0] == '#' ) continue;

    Vehicle truck( line, otherlocs );

    if ( truck.isvalid() ) {
      trucks.push_back( truck );
      depots.push_back( truck.getStartingSite() );
      dumps.push_back( truck.getDumpSite() );
      endings.push_back( truck.getEndingSite() );
    } else {
      invalidTrucks.push_back( truck );
    }
  }

  in.close();

}

#if 0
void Prob_trash::load_depots( std::string infile )
{
#ifdef VRPMINTRACE
  DLOG( INFO ) << "Prob_trash:Load_depots" << infile;
#endif
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;

  depots.clear();

  while ( getline( in, line ) ) {
    cnt++;

    if ( line[0] == '#' ) continue;

    Trashnode node( line );

    if ( not node.isValid() or not node.isDepot() ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "ERROR: line: " << cnt << ": " << line;
#endif
      invalid.push_back( node );
    } else {
      depots.push_back( node );
    }
  }

  in.close();
}
#endif

void Prob_trash::load_otherlocs( std::string infile )
{
#ifdef VRPMINTRACE
  DLOG( INFO ) << "Prob_trash:Load_otherlocs" << infile;
#endif
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;

  otherlocs.clear();

  while ( getline( in, line ) ) {
    cnt++;

    if ( line[0] == '#' ) continue;

    Trashnode node( line );

    if ( not node.isValid() ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "ERROR: line: " << cnt << ": " << line;
#endif
      invalid.push_back( node );
    } else {
      otherlocs.push_back( node );
    }
  }

  in.close();
}

#if 0
void Prob_trash::load_dumps( std::string infile )   //1 dump problem
{
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;
  dumps.clear();

  while ( getline( in, line ) ) {
    cnt++;

    // skip comment lines
    if ( line[0] == '#' ) continue;

    Trashnode node( line );

    if ( not node.isValid() or not node.isDump() ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "ERROR: line: " << cnt << ": " << line;
#endif
      invalid.push_back( node );
    } else {
      dumps.push_back( node );
    }
  }

  in.close();
}
#endif


void Prob_trash::load_pickups( std::string infile )
{
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;
  pickups.clear();
  double st, op, cl, dm, x, y;
  st = op = cl = dm = x = y = 0;

  while ( getline( in, line ) ) {
    cnt++;

    if ( line[0] == '#' ) continue;

    Trashnode node( line );
    node.set_type( Twnode::kPickup );

    if ( not node.isValid() ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "ERROR: line: " << cnt << ": " << line;
#endif
      invalid.push_back( node );
    } else {
      pickups.push_back( node );
      st += node.serviceTime();
      op += node.opens();
      cl += node.closes();
      dm += node.demand();
      x += node.x();
      y += node.y();
    }
  }

  in.close();
  st = st / pickups.size();
  op = op / pickups.size();
  cl = cl / pickups.size();
  dm = dm / pickups.size();
  x = x / pickups.size();
  y = y / pickups.size();
  C.set( -1, -1, x, y, dm, op, cl, st );
}

