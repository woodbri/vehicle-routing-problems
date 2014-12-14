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

#include "trashprob.h"

TrashProb::TrashProb(  container_t* p_containers, unsigned int container_count,
                          otherloc_t* p_otherlocs, unsigned int otherloc_count,
                          ttime_t* p_ttimes, unsigned int ttime_count,
                          vehicle_t* p_vehicles, unsigned int vehicle_count) {

    Bucket nodes;
    Bucket intersection;
    #ifdef VRPMINTRACE
    DLOG( INFO ) << "trashProb LoadProblem -------------from sql data---------";
    #endif


    // read the nodes
    int cnt = 0;
    int nid = 0;
    int id = 0;

    addContainers(p_containers, container_count);
    addOtherlocs( p_otherlocs, otherloc_count );

    intersection = otherlocs * pickups;
    invalid += intersection;
    pickups -= intersection;
    nodes -= intersection;

    #ifdef VRPMINTRACE
    invalid.dump( "invalid" );
    #endif


    nodes = pickups + otherlocs;
    nodes.push_back( C );

    for ( int i = 0; i < nodes.size(); i++ ) {
        nodes[i].setnid( i );
        id = nodes[i].getid();

        if ( pickups.hasId( id ) )
            pickups[ pickups.posFromId( id ) ].setnid( i );
        else if ( otherlocs.hasId( id ) )
            otherlocs[ otherlocs.posFromId( id ) ].setnid( i );
    };

    C = nodes.back();
    assert( pickups.size() );
    assert( otherlocs.size() );

    datanodes = nodes;
    twc->loadAndProcess_distance( p_ttimes, ttime_count, datanodes, invalid );
    addVehicles( p_vehicles, vehicle_count );

    twc->setHints( dumps );
    twc->setHints( nodes );
    twc->setHints( depots );
    twc->setHints( pickups );
    twc->setHints( endings );
    twc->settCC( C, pickups );


    assert( trucks.size() and depots.size() and dumps.size() and endings.size() );

    for ( int i = 0; i < trucks.size(); i++ ) {
        trucks[i].setInitialValues( C, pickups );
    }


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

    DLOG( INFO ) << "INVALID TRUCKS";

    for ( int i = 0; i < invalidTrucks.size(); i++ ) invalidTrucks[i].tau();

    twc->dump();
    #endif
}


void TrashProb::addContainers( container_t *_containers, int count ) {
    pickups.clear();
    double st, op, cl, dm, x, y;
    st = op = cl = dm = x = y = 0;

    for ( int i = 0; i < count; ++i ) {
        container_t c = _containers[i];
        Trashnode node( c.id, c.x, c.y, c.open, c.close, c.service, c.demand, c.sid ); //get it out of the cycle
        node.setType( 2 );

        if ( node.isValid() ) {
            pickups.push_back( node );
            st += node.getServiceTime();
            op += node.opens();
            cl += node.closes();
            dm += node.getDemand();
            x += node.getx();
            y += node.gety();
        }
        else invalid.push_back( node );
        
    }
    st = st / pickups.size();
    op = op / pickups.size();
    cl = cl / pickups.size();
    dm = dm / pickups.size();
    x = x / pickups.size();
    y = y / pickups.size();
    C.set( -1, -1, x, y, dm, op, cl, st );
}


void TrashProb::addOtherlocs( otherloc_t *_otherlocs, int count ) {
    otherlocs.clear();

    for ( int i = 0; i < count; ++i ) {
        otherloc_t c = _otherlocs[i];
        Trashnode node( c.id, c.x, c.y, c.open, c.close, 0, 0, -1 );

        if ( node.isValid() ) {
            otherlocs.push_back( node );
        }
        else {
            invalid.push_back( node );
        }
    }
}





void TrashProb::addVehicles( vehicle_t *_vehicles, int count ) {
    for ( int i = 0; i < count; ++i ) {
        vehicle_t v = _vehicles[i];
        Vehicle truck( v.vid, v.start_id, v.dump_id, v.end_id, v.capacity,
                       v.dumpservicetime, v.starttime, v.endtime, otherlocs );

        if ( truck.isvalid() ) {
            trucks.push_back( truck );
            depots.push_back( truck.getStartingSite() );
            dumps.push_back( truck.getDumpSite() );
            endings.push_back( truck.getEndingSite() );
        }
        else {
            invalidTrucks.push_back( truck );
        }
    }

}


bool TrashProb::isValid() const {
    return trucks.size()
           and depots.size()
           and dumps.size()
           and endings.size()
           and pickups.size()
           and otherlocs.size()
	   and not invalid.size()
	   and not invalidTrucks.size();
           ;
}

std::string TrashProb::whatIsWrong() const {
    std::ostringstream wiw( std::ostringstream::ate );

    if ( not trucks.size() ) wiw << "No valid vehicles\n";

    if ( not depots.size() ) wiw << "No valid starting locations found\n";

    if ( not dumps.size() ) wiw << "No valid dumps found\n";

    if ( not endings.size() ) wiw << "No valid ending locations found\n";

    if ( not pickups.size() ) wiw << "No valid container locations found\n";

    if ( not otherlocs.size() ) wiw << "No valid other locations found\n";

    if ( invalid.size() ) {
        wiw << "The following nodes are invalid: ";

        for ( int i = 0; i < invalid.size(); ++i )
            wiw << invalid[i].getid() << " ";

        wiw << "\n";
    }

    if ( invalidTrucks.size() ) {
        wiw << "The following vehicles are invalid: ";

        for ( int i = 0; i < invalidTrucks.size(); ++i )
            wiw << invalidTrucks[i].getVid() << " ";

        wiw << "\n";
    }

    return wiw.str();
}



