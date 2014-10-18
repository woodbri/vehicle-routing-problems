
#include "trashprob.h"


void TrashProb::addContainers( container_t *_containers, int count ) {
    pickups.clear();
    for (int i=0; i<count; ++i) {
        container_t c = _containers[i];
        Trashnode node(c.id, c.x, c.y, c.open, c.close, c.service, c.demand, c.sid);
        if (node.isvalid()) {
            pickups.push_back(node);
        }
        else {
            invalid.push_back(node);
        }
    }
}


void TrashProb::addOtherlocs( otherloc_t *_otherlocs, int count ) {
    otherlocs.clear();
    for (int i=0; i<count; ++i) {
        otherloc_t c = _otherlocs[i];
        Trashnode node(c.id, c.x, c.y, c.open, c.close, 0, 0, -1);
        if (node.isvalid()) {
            otherlocs.push_back(node);
        }
        else {
            invalid.push_back(node);
        }
    }
}


bool TrashProb::checkNodesOk() {
    if (pickups.size() == 0) return false;
    if (otherlocs.size() == 0) return false;

    Bucket nodes;
    Bucket intersection;
    int id;

    intersection = otherlocs * pickups;
    invalid += intersection;
    pickups -= intersection;
    nodes = pickups + otherlocs;

    for (int i=0; i<nodes.size(); i++) {
        nodes[i].setnid(i);
        id = nodes[i].getid();
        if ( pickups.hasId(id) ) pickups[ pickups.posFromId(id) ].setnid(i);
        else if ( otherlocs.hasId( id ) ) otherlocs[ otherlocs.posFromId( id ) ].setnid(i);
    }

    if (not pickups.size() or not otherlocs.size()) return false;

    datanodes=nodes;

    return true;
}


void TrashProb::addTtimes( ttime_t *_ttimes, int count ) {

    twc.loadAndProcess_distance(_ttimes, count, datanodes, invalid);

    Bucket dummy;
    dummy.setTravelTimes(twc.TravelTime());
    Tweval dummyNode;
    dummyNode.setTravelTimes(twc.TravelTime());
//    assert( Tweval::TravelTime.size() );
}


void TrashProb::addVehicles( vehicle_t *_vehicles, int count ) {
    for (int i=0; i<count; ++i) {
        vehicle_t v = _vehicles[i];
        Vehicle truck(v.vid, v.start_id, v.dump_id, v.end_id, v.capacity,
            v.dumpservicetime, v.starttime, v.endtime, otherlocs);
        if (truck.isvalid()) {
            trucks.push_back(truck);
            depots.push_back(truck.getStartingSite());
            dumps.push_back(truck.getdumpSite());
            endings.push_back(truck.getEndingSite());
        }
        else {
            invalidTrucks.push_back(truck);
        }
    }
//    assert(trucks.size() and depots.size() and dumps.size() and endings.size());
}


bool TrashProb::isValid() const {
    return trucks.size()
       and depots.size()
       and dumps.size()
       and endings.size()
       and pickups.size()
       and otherlocs.size()
       and Tweval::TravelTime.size()
       ;
}

std::string TrashProb::whatIsWrong() const {
    std::ostringstream wiw(std::ostringstream::ate);
    if (not trucks.size())
        wiw << "No valid vehicles\n";
    if (not depots.size())
        wiw << "No valid starting locations found\n";
    if (not dumps.size())
        wiw << "No valid dumps found\n";
    if (not endings.size())
        wiw << "No valid ending locations found\n";
    if (not pickups.size())
        wiw << "No valid container locations found\n";
    if (not otherlocs.size())
        wiw << "No valid other locations found\n";
    if (not Tweval::TravelTime.size())
        wiw << "The travel time matrix is empty\n";
    if (invalid.size()) {
        wiw << "The following nodes are invalid: ";
        for (int i=0; i<invalid.size(); ++i)
            wiw << invalid[i].getid() << " ";
        wiw << "\n";
    }
    if (invalidTrucks.size()) {
        wiw << "The following vehicles are invalid: ";
        for (int i=0; i<invalidTrucks.size(); ++i)
            wiw << invalidTrucks[i].getVid() << " ";
        wiw << "\n";
    }

    return wiw.str();
}



