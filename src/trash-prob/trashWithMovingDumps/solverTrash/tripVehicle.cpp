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
  DLOG(INFO) << "manual control:";
  UINT del_node, del_pos, del_trip;
  UINT ins_after, ins_pos, ins_trip;
  double del_delta, ins_delta;
  del_trip = 2;
  ins_trip = 1;
  trips[del_trip].bestRemoval(del_node, del_pos, del_delta);
  DLOG(INFO) << "best removal trip:" << del_trip
             << " at pos: " << del_pos
             << "node:" << trips[del_trip][del_pos].id()
             << "delta:" << del_delta;

  trips[ins_trip].bestInsertion(del_node, ins_after, ins_pos, ins_delta);
  DLOG(INFO) << "best insertion trip:" << ins_trip
             << " at pos: " << ins_pos
             << "after node:" << trips[ins_trip][ins_pos-1].id()
             << "delta:" << del_delta;
  
  assert(true == false);
  tauTrips();
}

// return the deltaTime
void Trip::bestRemoval(UINT &best, POS &pos, double &btime) {
  assert(size() > 1); // if size == 1 the trip shoudnt exist
  tau("bestRemoval");
  getCostOsrm();
  evaluate();
  pos = 1;
  if (size()==1) {
    best = path[1].nid();
    btime = -99999;
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
      DLOG(INFO) << "old best " << btime;
      DLOG(INFO) << "new best " << deltaTime;
      btime = deltaTime;
      pos = i;
    }
  }
  best = path[pos].nid();
DLOG(INFO) << "best removal" << path[pos].id();;
  return;
}

void Trip::bestInsertion(UINT n_ins, UINT &ins_aft, POS &ins_pos, double &btime) {
  tau("bestInsertion");
  assert(size() > 1); // will never use insert into an empty truck
  ins_pos = 1;
  btime = 999999;
  double time0, time1, deltaTime;
  for (UINT i = 1; i < path.size(); ++i) {
    DLOG(INFO) << "in cycle" << i;
    if (i == path.size()-1) {
      if (twc->isInPath(path[i].nid(), n_ins, dumpSite.nid())) {
        // cool, fits perfectly
        ins_pos = i;
        ins_aft = path[i-1].nid();
        btime = 0;
        return;
      }
      time1 = twc->TravelTime(path[i].nid(), n_ins, dumpSite.nid());
      DLOG(INFO) << "new Time" << path[i].id() << "->* " << n_ins << "->" << dumpSite.id() << " =  " << time1;
      time0 = twc->TravelTime(path[i].nid(), dumpSite.nid());
      DLOG(INFO) << "old Time" << path[i].id() << "->" << dumpSite.id() << "=  " << time0;
    } else {
      if (twc->isInPath(path[i-1].nid(), n_ins, path[i].nid())) {
        // cool, fits perfectly
        ins_pos = i;
        ins_aft = path[i-1].nid();
        btime = 0;
        return;
      }
      time1 = twc->TravelTime(path[i-1].nid(), n_ins, path[i].nid());
      DLOG(INFO) << "new Time" << path[i-1].id() << "->* " << n_ins << "->" << path[i].id() <<" =  " << time1;
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid());
      DLOG(INFO) << "old Time" << path[i-1].id() << "->" <<  path[i].id() << " = " << time0;
    }
    deltaTime = time1 - time0;
    DLOG(INFO) << "delta TIme = new -old " << deltaTime;
    if (deltaTime < btime) {
      btime = deltaTime;
      ins_pos = i;
      ins_aft = path[i-1].id();
    }
  }
DLOG(INFO) << "best insertion after" << ins_aft;
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
manualControl();


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
    trips.push_back(trip); 
    
}
