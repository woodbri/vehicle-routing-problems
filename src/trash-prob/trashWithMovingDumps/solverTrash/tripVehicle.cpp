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
  swapBestToDump();
  intraTripOptimizationNoOsrm();

  POS  del_pos, ins_pos;
  POS  o_del_pos, o_ins_pos;
  UINT  trip, o_trip;
  double delta1, delta2;
  trip = 2; 
  o_trip = 1;

  exchanges(2,0,10);
  exchanges(2,1,10);
  exchanges(1,0,10);
  exchanges(2,0,10);
  exchanges(2,1,10);
  exchanges(1,0,10);

#if 0
  for (UINT i=0; i< 10; ++i) {
    exchanges(2,1);
  }
  for (UINT i=0; i< 10; ++i) {
    exchangesWithOnPath(2,1);
  }
// #else
  exchangesWithOnPath(1,0);
  exchanges(2,1);
  exchangesWithOnPath(2,1);
  exchanges(1,0);
  exchangesWithOnPath(1,0);
#endif
  tauTrips();
  assert(true == false);

}

void Vehicle::intraTripOptimizationNoOsrm() {
  tauTrips();
  for (auto &trip : trips) {
    trip.intraTripOptimizationNoOsrm();
  }
}

void Vehicle::exchangesWithOnPath(UINT trip, UINT o_trip) {
  POS  del_pos, ins_pos;
  POS  o_del_pos, o_ins_pos;
  double delta_del, delta_ins;
  double o_delta_del, o_delta_ins;
  bool inPath1, inPath2;

  for (UINT i=0; i< 1; ++i) {
    tauTrips();
    inPath1 = trips[trip].getRemovalValues(trips[o_trip], o_ins_pos, del_pos, delta_del, o_delta_ins);
// bool Trip::chooseMyBest(const Trip &other, POS o_ins_pos, POS del_pos, POS &ins_pos, POS &o_del_pos, double &o_delta_del, double &delta_ins) const {

    inPath2 = trips[trip].chooseMyBest(trips[o_trip], o_ins_pos, del_pos, ins_pos, o_del_pos, o_delta_del, delta_ins);
    if (inPath2) {
       DLOG(INFO) <<  "-> with on trips delta shortest " << delta_del + delta_ins ; 
       DLOG(INFO) <<  "delta total " <<   delta_del + delta_ins + o_delta_del + o_delta_ins;
       DLOG(INFO) <<  "in path trip " << trip << "\tnode " << trips[trip][del_pos].id() << " to trip " << o_trip << "\t after " << trips[o_trip][o_ins_pos-1].id();
       DLOG(INFO) <<  "in path trip " << o_trip << "\tnode " << trips[o_trip][o_del_pos].id() << " to trip " << trip << "\t after " << trips[trip][ins_pos-1].id();
       trips[trip].exchange(trips[o_trip], del_pos, o_ins_pos, o_del_pos, ins_pos);
    } else {
      break;
    }
  }
}

void Vehicle::exchanges(UINT trip, UINT o_trip, int lim_iter) {
  for (UINT i=0; i< lim_iter; ++i) {
    if (!exchange(trip, o_trip)) break;
  }
}


bool Vehicle::exchange(UINT trip, UINT o_trip) {
  POS  del_pos, ins_pos;
  POS  o_del_pos, o_ins_pos;
  double delta_del, delta_ins;
  double o_delta_del, o_delta_ins;
  bool inPath1, inPath2;

  inPath1 = trips[trip].getRemovalValues(trips[o_trip], o_ins_pos, del_pos, delta_del, o_delta_ins);
  trips[o_trip].getRemovalValues(trips[trip], ins_pos, o_del_pos, o_delta_del, delta_ins);
  if ( delta_del + delta_ins + o_delta_del + o_delta_ins <= 0) {
     DLOG(INFO) <<  "delta shortest " << delta_del + delta_ins ;
     DLOG(INFO) <<  "delta total " <<  delta_del + delta_ins + o_delta_del + o_delta_ins;
     DLOG(INFO) <<  "trip " << trip << "\tnode " << trips[trip][del_pos].id() << " to trip " << o_trip << "\t after " << trips[o_trip][o_ins_pos-1].id();
     DLOG(INFO) <<  "trip " << o_trip << "\tnode " << trips[o_trip][o_del_pos].id() << " to trip " << trip << "\t after " << trips[trip][ins_pos-1].id();
     trips[trip].exchange(trips[o_trip], del_pos, o_ins_pos, o_del_pos, ins_pos);
     return true;
  }
  return false;
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
  // tau("this after insert");
  // other.tau("other after insert");
  
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
// true when the removal of this can be inserted in a path of the other
bool Trip::getRemovalValues(const Trip &other, POS &o_ins_pos, POS &del_pos, double &o_delta_ins, double &delta_del) const{
  UINT del_node;
  UINT ins_after;
  bool insertInPath;
  bestRemoval(del_node, del_pos, delta_del);
  insertInPath = other.bestInsertion(del_node, ins_after, o_ins_pos, o_delta_ins);

//  tau("best removal trip:");
  DLOG(INFO) << "\tnode to be removed: " << path[del_pos].id()
             << " at pos: " << del_pos
             << "\tdelta: " << delta_del;
  DLOG(INFO) << "\tinsert after node: " << other[o_ins_pos-1].id()
             << " at pos: " << o_ins_pos
             << "\tdelta: " << o_delta_ins;
  double delta = o_delta_ins + delta_del;
  return delta < 0;
}


// I want node from other
// special case when  best inserted is in  del_pos
// this: S 0 1 2 3 4 5 6 7
//             |
//             d

// nor it can not be from o_ins_pos sorroundings
// other: S A B (C D)  E F G
//                 |
//                o_i

// in the example C D  are forbidden 

// So I search on this nodes
// o_nodes: A B F G
// suppose this nodes from the other trip are on my path
// nodesOnPath:  B D F

bool Trip::chooseMyBest(const Trip &other, POS o_ins_pos, POS del_pos, POS &ins_pos, POS &o_del_pos, double &o_delta_del, double &delta_ins) const {
  Bucket o_nodes;
  Bucket nodesOnPath;  // the nodes on my path
  o_nodes = other.path; // choose from this
  if (o_ins_pos-1 < o_nodes.size()) o_nodes.erase(o_ins_pos-1); // this nodes the other cant give
  if (o_ins_pos-1 < o_nodes.size()) o_nodes.erase(o_ins_pos-1);
  o_nodes.pop_front();  // delete the starting site
  twc->getNodesOnPath(path, dumpSite, o_nodes, nodesOnPath);
  o_nodes = o_nodes - nodesOnPath;
  o_nodes.dumpid("nodes not in my path");
  nodesOnPath.dumpid("nodes in my path");
  auto found = false;
  UINT o_del_node, ins_after;


  double time0, time1, deltaTime;
  if (nodesOnPath.size() > 0) {
    o_del_node = nodesOnPath[0].nid();
    o_del_pos = other.path.pos(o_del_node);
    o_delta_del = 999999;
    for(auto j = 0; j < nodesOnPath.size(); ++j) {
      UINT node = nodesOnPath[j].nid(); // working with node
      POS pos_o = other.path.pos(node); // located at this postition in the others path
      if (o_del_pos == other.path.size()-1) { // its the last node
        time0 = twc->TravelTime(other[pos_o - 1].nid(), other[pos_o].nid(), other.dumpSite.nid());
        time1 = twc->TravelTime(other[pos_o - 1].nid(), other.dumpSite.nid());
      } else {
        time0 = twc->TravelTime(other[pos_o - 1].nid(), other[pos_o].nid(), other[pos_o + 1].nid());
        time1 = twc->TravelTime(other[pos_o - 1].nid(), other[pos_o+ 1].nid());
      } 
      deltaTime = time1 - time0; 
      if (deltaTime < o_delta_del) {
        // its the best removal at the moment, so I check its not inserted in wrong place
        bestInsertion(o_del_node, ins_after, ins_pos, delta_ins);
        // special postion (del_pos) calculate the delta_ins ?????
        o_delta_del = deltaTime;
        o_del_pos = pos_o;
        o_del_node = node;
        found = true;
      }
    }
    if (found) {
      return true;
      // we got here because we found a node from the other trip that is in my path
    }
    assert(true==false);
  }
}

double  Trip::delta_del(POS del_pos) const {
  assert(del_pos > 0 && del_pos < path.size());
  double time0, time1;
  if (del_pos == path.size()-1) { // its the last node
    if (twc->isInPath(path[del_pos - 1], path[del_pos], dumpSite)) return 0; 
    time0 = twc->TravelTime(path[del_pos - 1].nid(), path[del_pos].nid(), dumpSite.nid());
    time1 = twc->TravelTime(path[del_pos - 1].nid(), dumpSite.nid());
  } else {
    if (twc->isInPath(path[del_pos - 1], path[del_pos], path[del_pos + 1])) return 0;
    time0 = twc->TravelTime(path[del_pos - 1].nid(), path[del_pos].nid(), path[del_pos + 1].nid());
    time1 = twc->TravelTime(path[del_pos - 1].nid(), path[del_pos+ 1].nid());
  } 
  return time1 - time0;
}


#if 0
    // DLOG(INFO) << nodesOnPath[j].nid() <<","<<other[o_del_pos].nid();
    // bestInsertion(o_del_node, ins_after, ins_pos, delta_ins);


      if (j == 0) {
        o_del_node = nodesOnPath[j].nid();
        o_del_pos = other.path.pos(o_del_node);
        DLOG(INFO) << nodesOnPath[j].nid() <<","<<other[o_del_pos].nid();
        bestInsertion(o_del_node, ins_after, ins_pos, delta_ins);
      }

      // to do find the "best one" what is best in this case?
#endif








// return the deltaTime
void Trip::bestRemoval(UINT &d_node, POS &d_pos, double &d_delta) const {
  assert(size() > 1); // if size == 1 the trip shoudnt exist
  //tau("bestRemoval");
  // getCostOsrm();
  // evaluate();
  d_pos = 1;
  if (size()==1) {
    d_node = path[1].nid();
    d_delta = delta_del(1);
// DLOG(INFO) << "best removal" << path[pos].id();;
    return;
  }
  double deltaTime;
  d_delta = 99999;
  for (UINT i = 1; i < path.size()-1; ++i) {
#if 0
    if (i == path.size()-1) {
      if (twc->isInPath(path[i-1], path[i], dumpSite)) continue;
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid(), dumpSite.nid());
      time1 = twc->TravelTime(path[i-1].nid(), dumpSite.nid());
    } else {
      if (twc->isInPath(path[i-1], path[i], path[i+1])) continue;
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid(), path[i+1].nid());
      time1 = twc->TravelTime(path[i-1].nid(), path[i+1].nid());
    }
#endif
    deltaTime = delta_del(i); 

    if (d_delta > deltaTime) {
      d_delta = deltaTime;
      d_pos = i;
      d_node = path[d_pos].nid();
    }
  }
  return;
}

bool Trip::bestInsertion(UINT n_ins, UINT &ins_aft, POS &ins_pos, double &delta_ins) const {
  assert(size() > 1); // will never use insert into an empty truck
  ins_pos = 1;
  delta_ins = 999999;
  double time0, time1, deltaTime;
  for (UINT i = 1; i < path.size(); ++i) {
    if (i == path.size()-1) {
      if (twc->isInPath(path[i].nid(), n_ins, dumpSite.nid())) {
        ins_pos = i;
        ins_aft = path[i-1].nid();
        delta_ins = 0;
        return true;
      }
      time1 = twc->TravelTime(path[i].nid(), n_ins, dumpSite.nid());
      time0 = twc->TravelTime(path[i].nid(), dumpSite.nid());
    } else {
      if (twc->isInPath(path[i-1].nid(), n_ins, path[i].nid())) {
        ins_pos = i;
        ins_aft = path[i-1].nid();
        delta_ins = 0;
        return true;
      }
      time1 = twc->TravelTime(path[i-1].nid(), n_ins, path[i].nid());
      time0 = twc->TravelTime(path[i-1].nid(), path[i].nid());
    }
    deltaTime = time1 - time0;
    if (deltaTime < delta_ins) {
      delta_ins = deltaTime;
      ins_pos = i;
      ins_aft = path[i-1].nid();
    }
  }
  return false;
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
  DLOG(INFO) << "trips";
  for (auto &trip : trips) {
    trip.tau();
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
    trip.trip_id() = trips.size();
    trips.push_back(trip); 
}
