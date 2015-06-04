#include <algorithm>
#include "tripVehicle.h"



void Trip::swapBestToDump(Trip &other) {
  // this trips wants to get faster to the dump
  // this is common bewtween all trips
  if (twc->TravelTime(other.last(),other.dumpSite)
      < twc->TravelTime(this->last(),this->dumpSite)) { 
     std::swap(other.path[other.size()-1], path[this->size()-1]);
  }
}



void Vehicle::manualControl() {

  swapBestToDump();

  DLOG(INFO) << "manual control:";
  POS  del_pos, ins_pos;
  POS  o_del_pos, o_ins_pos;
  UINT  trip, o_trip;
  double delta1, delta2;
  trip = 2; 
  o_trip = 1;
#if 0
  trips[del_trip1].bestRemoval(del_node1, del_pos1, del_delta1);
  DLOG(INFO) << "best removal trip:" << del_trip1
             << " at pos: " << del_pos1
             << "node:" << trips[del_trip1][del_pos1].id()
             << "delta:" << del_delta1;

  trips[ins_trip1].bestInsertion(del_node1, ins_after1, ins_pos1, ins_delta1);
  DLOG(INFO) << "best insertion trip:" << ins_trip1
             << " at pos: " << ins_pos1
             << "after node:" << trips[ins_trip1][ins_pos1-1].id()
             << "delta:" << ins_delta1;
#endif
  trips[0].tau("trip 0 not used");
 
  for (UINT i=0; i< 10; ++i) {
  trips[trip].tau("trip 2");
  trips[o_trip].tau("trip 1");
  trips[trip].getRemovalValues(trips[o_trip], o_ins_pos, del_pos, delta1);
  trips[o_trip].getRemovalValues(trips[trip], ins_pos, o_del_pos, delta2);
  DLOG(INFO) << "deltas" << delta1 + delta2;
  if (delta1 + delta2 > 0) break;
  trips[trip].exchange(trips[o_trip], del_pos, o_ins_pos, o_del_pos, ins_pos);
  trips[trip].tau("trip 2");
  trips[o_trip].tau("trip 1");
  }

  trip = 1; 
  o_trip = 0;
  for (UINT i=0; i< 10; ++i) {
  trips[trip].tau("trip 1");
  trips[o_trip].tau("trip 0");
  trips[trip].getRemovalValues(trips[o_trip], o_ins_pos, del_pos, delta1);
  trips[o_trip].getRemovalValues(trips[trip], ins_pos, o_del_pos, delta2);
  DLOG(INFO) << "deltas" << delta1 + delta2;
  if (delta1 + delta2 > 0) break;
  trips[trip].exchange(trips[o_trip], del_pos, o_ins_pos, o_del_pos, ins_pos);
  trips[trip].tau("trip 1");
  trips[o_trip].tau("trip 0");
  }



  assert(true == false);
  tauTrips();
}


void Trip::exchange(Trip &other,
    POS del_pos,   // the node I am going to give to the other
    POS o_ins_pos, // where it is going to be placed in the other
    POS o_del_pos, // the node the other is going to give me
    POS ins_pos) {    // where I am going to insert it
  // D 0 1 2 3 4 5 6 7 8 9
  //         |       |
  //        del     ins
  // D A B C D E F G H I J
  //     |       |
  //    o_ins   o_del

  // AFTER
  // D 0 1 2 4 5 6 F 7 8 9
  // note: 3 is deleted and F is inserted after ins-1

  // D A 3 B C D E G H I J
  // note: F is deleted and 3 is inserted after o_ins-1

  // save the nodes because they are going to be shifted
  Trashnode myNode = path[del_pos]; 
  Trashnode hisNode = other.path[o_del_pos]; 

  tau("this before insert");
  other.tau("other before insert");
  // D 0 1 3 2 4 5 6 F 7 8 9
  path.insert(hisNode, ins_pos);
  // D A 3 B C D E F G H I J
  other.path.insert(myNode, o_ins_pos);
  tau("this after insert");
  other.tau("other after insert");
  
  //calculate the delete position
  del_pos = del_pos < ins_pos? del_pos: ++del_pos;
  o_del_pos = o_del_pos < o_ins_pos? o_del_pos: ++o_del_pos;
  path.erase(del_pos);
  other.path.erase(o_del_pos);
  tau("this after remove");
  other.tau("other after remove");

}

// this trip wants to remove
// asks for values to other trip

bool Trip::getRemovalValues(const Trip &other, POS &o_ins_pos, POS &del_pos, double &delta) const{
  UINT del_node;
  UINT ins_after;
  double del_delta, ins_delta;
  bestRemoval(del_node, del_pos, del_delta);
  other.bestInsertion(del_node, ins_after, o_ins_pos, ins_delta);

//  tau("best removal trip:");
  DLOG(INFO) << "\tnode to be removed:" << path[del_pos].id()
             << " at pos: " << del_pos
             << "\tdelta:" << del_delta;
  DLOG(INFO) << "\tinsert after node:" << other[o_ins_pos-1].id()
             << " at pos: " << o_ins_pos
             << "\tdelta:" << ins_delta;
  delta = del_delta + ins_delta;
  return delta < 0;
}









// return the deltaTime
void Trip::bestRemoval(UINT &best, POS &pos, double &btime) const {
  assert(size() > 1); // if size == 1 the trip shoudnt exist
  //tau("bestRemoval");
  // getCostOsrm();
  // evaluate();
  pos = 1;
  if (size()==1) {
    best = path[1].nid();
    btime = -99999;
// DLOG(INFO) << "best removal" << path[pos].id();;
    return;
  }
  double time0, time1, deltaTime;
  for (UINT i = 1; i < path.size()-1; ++i) {
    if (i == path.size()-1) {
      if (twc->isInPath(path[i-1], path[i], dumpSite)) continue;
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid(), dumpSite.nid());
     // DLOG(INFO) << "old Time" << path[i-1].id() << "->* " << path[i].id() << "->" << dumpSite.id() << " =  " << time0;
      time1 = twc->TravelTime(path[i-1].nid(), dumpSite.nid());
     // DLOG(INFO) << "new Time" << path[i-1].id() << "->" << dumpSite.id() << "=  " << time1;
    } else {
      if (twc->isInPath(path[i-1], path[i], path[i+1])) continue;
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid(), path[i+1].nid());
     // DLOG(INFO) << "old Time" << path[i-1].id() << "->* " << path[i].id() << "->" << path[i+1].id() <<" =  " << time0;
      time1 = twc->TravelTime(path[i-1].nid(), path[i+1].nid());
     // DLOG(INFO) << "new Time" << path[i-1].id() << "->" <<  path[i+1].id() << " = " << time1;
    }
    deltaTime = time1 - time0; 
    #if 0
    DLOG(INFO) << "delta TIme = new -old " << deltaTime;
    if (deltaTime > 0) {
      DLOG(INFO) << "removing the container increments the time";
      continue;
    } else {
      DLOG(INFO) << "  old Time" << path[i-1].id() << " " << path[i].id() << " other" << " =  " << time0;
      DLOG(INFO) << "  new Time" << path[i-1].id() << " other"  << "=  " << time1;
   }
   #endif

    if (deltaTime < btime) {
    //  DLOG(INFO) << "old best " << btime;
    //  DLOG(INFO) << "new best " << deltaTime;
      btime = deltaTime;
      pos = i;
    }
  }
  best = path[pos].nid();
// DLOG(INFO) << "best removal" << path[pos].id();;
  return;
}

void Trip::bestInsertion(UINT n_ins, UINT &ins_aft, POS &ins_pos, double &btime) const {
  //tau("bestInsertion");
  assert(size() > 1); // will never use insert into an empty truck
  ins_pos = 1;
  btime = 999999;
  double time0, time1, deltaTime;
  for (UINT i = 1; i < path.size(); ++i) {
    // DLOG(INFO) << "in cycle" << i;
    if (i == path.size()-1) {
      if (twc->isInPath(path[i].nid(), n_ins, dumpSite.nid())) {
        // cool, fits perfectly
        ins_pos = i;
        ins_aft = path[i-1].nid();
        btime = 0;
// DLOG(INFO) << "best insertion after " << path[ins_pos-1].id();
        return;
      }
      time1 = twc->TravelTime(path[i].nid(), n_ins, dumpSite.nid());
      // DLOG(INFO) << "new Time" << path[i].id() << "->* " << n_ins << "->" << dumpSite.id() << " =  " << time1;
      time0 = twc->TravelTime(path[i].nid(), dumpSite.nid());
      // DLOG(INFO) << "old Time" << path[i].id() << "->" << dumpSite.id() << "=  " << time0;
    } else {
      if (twc->isInPath(path[i-1].nid(), n_ins, path[i].nid())) {
        // cool, fits perfectly
        ins_pos = i;
        ins_aft = path[i-1].nid();
        btime = 0;
// DLOG(INFO) << "best insertion after " << path[ins_pos-1].id();
        return;
      }
      time1 = twc->TravelTime(path[i-1].nid(), n_ins, path[i].nid());
      // DLOG(INFO) << "new Time" << path[i-1].id() << "->* " << n_ins << "->" << path[i].id() <<" =  " << time1;
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid());
      // DLOG(INFO) << "old Time" << path[i-1].id() << "->" <<  path[i].id() << " = " << time0;
    }
    deltaTime = time1 - time0;
    // DLOG(INFO) << "delta TIme = new -old " << deltaTime;
    if (deltaTime < btime) {
      btime = deltaTime;
      ins_pos = i;
      ins_aft = path[i-1].nid();
    }
  }
// DLOG(INFO) << "best insertion after " << path[ins_pos-1].id();
  return;
}

    
    
  



void Vehicle::swapBestToDump() {
  DLOG(INFO) << "before swap";
  tauTrips();
  for (UINT i = 0; i < trips.size()-1; ++i) {
    trips[i+1].swapBestToDump(trips[i]);
  }
  DLOG(INFO) << "after swap";
  tauTrips();
}


void Vehicle::tauTrips() const {
  for (UINT i = 0; i < trips.size()-1; ++i) {
    trips[i].tau();
  }
}

Trip Vehicle::get_new_trip() {
  DLOG(INFO) << "get_new_trip";
  Trashnode S,D,E;
  if (path.size()==1) {
    S = path[0];
    D = dumpSite;
    E = endingSite;
  DLOG(INFO) << "truck size is 1";
  } else {
  DLOG(INFO) << "truck size is not 1";
  tau("truck");
assert(true==false);
    S = dumpSite;
    D = dumpSite;
    E = endingSite;
  } 
#if 0
  DLOG(INFO) << "truck short eval";
  print_short_eval();

  DLOG(INFO) << "going to initialize";
#endif
  Trip trip(S, D, E, maxcapacity);
#if 0
  DLOG(INFO) << "trips short eval";
  trip.print_short_eval();
  assert(true==false);
#endif
  return trip;
}
    


void Vehicle::add_trip(const Trip &p_trip) {

    Trip trip = p_trip;
    tau("before inserting trip");
    this->e_add_trip(trip);
    tau("after inserting trip");
    // the start of the trip does not change
    // the dump of the trip must have the same values as the dump of the truck
    trip.getCostOsrm();
    getCostOsrm();
    trip.print_short_eval();
    print_short_eval();
    //need to adjust trips values    

    // need to add the trip in the trips
    // because they are ordered we need to make sure
    // that trip 0 has 100001 as start
    // that the others have dump as start

    // that the trip inserted has ending as the last
    // that the other trips have dump as ending
    trips.push_back(trip); 
    
}
