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
  // swapBestToDump();
  tauTrips();
#if 0
  trips[2].intraTripOptimizationNoOsrm();
  tauTrips();
  trips[2].intraTripOptimizationNoOsrm();
assert(true==false);
#endif

  tauTrips();
  intraTripOptimizationNoOsrm();
  tauTrips();
  assert(true==false);

  POS  del_pos, ins_pos;
  POS  o_del_pos, o_ins_pos;
  UINT  trip, o_trip;
  double delta1, delta2;
  trip = 2; 
  o_trip = 1;
  auto count = 0;
  auto tot_count = 0;
  
  DLOG(INFO) << "start exchanges" ;
  tauTrips();

#if 0
  for (UINT i = 0; i < 2; ++i) {
    count = exchangesWorse(10);
    tot_count += count;
    DLOG(INFO) << "exchangeAllWorse Performed: " << count;
    //if (count == 0) break;
  }
  DLOG(INFO) << "total exchangeAllWorse Performed: " << count;
#endif

  for (UINT i = 0; i < 5; ++i) {
    for (auto &trip : trips) {
      for (auto &o_trip : trips) {
        DLOG(INFO) << "exchanges " << trip.trip_id() << " , " << o_trip.trip_id();
        exchangesWithOnPath(trip, o_trip);
 //       intraTripOptimizationNoOsrm();
//        exchangesWithOnPath(o_trip, trip);
      }
    }
  }
//  intraTripOptimizationNoOsrm();


  tauTrips();
//TODO reconstruct the truck
  //assert(true == false);

}


void Vehicle::intraTripOptimizationNoOsrm() {
  tauTrips();
  for (auto &trip : trips) {
    trip.intraTripOptimizationNoOsrm();
  }
}

void Trip::intraTripOptimizationNoOsrm() {
  getCostOsrm();
  bool oldStateOsrm = osrmi->getUse();
  osrmi->useOsrm(false);
  POS i_pos;
  double i_delta;

  Trip trip = *this;
#if 0
  while (trip.size() > 3) {
    trip.e_remove(2);
  }
  trip.tau("only the begining and the end");
  e_remove(1);
  path.pop_back();
#endif

  Bucket nodesOnTripPath;
//////////////////////////////////////////////////////////////////////
  // instead of this:
  trip.getNodesOnPath(*this, nodesOnTripPath);

  
  nodesOnTripPath.dumpid("nodes on path");
//////////////////////////////////////////////////////////////////////
  while (nodesOnTripPath.size() > 0) { 
    auto siz = nodesOnTripPath.size() - 1;
    auto node = nodesOnTripPath[siz];
    auto d_node = nodesOnTripPath[siz].nid();
    trip.bestInsertion(d_node, i_pos, i_delta);
    //DLOG(INFO) << "node " << node.id() <<  "\t inserted in " << i_pos << " after " << trip[i_pos - 1].id();
    trip.path.insert(node, i_pos);
    //trip.tau("after");
    nodesOnTripPath.pop_back();
    path.erase(node);
  }
 
// by getting the worse and isnerting
#if 0
  while (path.size() > 1) {
  double d_delta;
  POS d_pos;
  UINT d_node;
  bestRemoval(d_node, d_pos, d_delta);
  auto node = path[d_pos];
  path.erase(d_pos);
  trip.bestInsertion(d_node, i_pos, i_delta);
  trip.path.insert(node, i_pos);
  DLOG(INFO) << "node " << node.id() <<   "\t inserted in " << i_pos << " after " << trip[i_pos - 1].id();
  trip.tau("after");
  }
#endif

#if 1
// in order from beginning to end
  while (path.size() > 1) {
  double d_delta;
  POS d_pos = 1;
  auto node = path[d_pos];
  UINT d_node = node.nid();;
  path.erase(d_pos);
  trip.bestInsertion(d_node, i_pos, i_delta);
  trip.path.insert(node, i_pos);
  //DLOG(INFO) << "node " << node.id() <<   "\t inserted in " << i_pos << " after " << trip[i_pos - 1].id();
  //trip.tau("after");
  }

// STEVE TEST trip.orderNodesAlongPath(orderedNodes);
  Bucket orderedNodes;
  trip.orderNodesAlongPath(orderedNodes);
  assert(true==false);
// END STEVE TEST


  path = trip.path;
#else
// in order from end to beginning 
  while (path.size() > 1) {
  double d_delta;
  POS d_pos = path.size()-1;
  auto node = path[d_pos];
  UINT d_node = node.nid();;
  path.erase(d_pos);
  trip.bestInsertion(d_node, i_pos, i_delta);
  trip.path.insert(node, i_pos);
  DLOG(INFO) << "node " << node.id() <<   "\t inserted in " << i_pos << " after " << trip[i_pos - 1].id();
  trip.tau("after");
  }
#endif
 // assert(true==false);
  

#if 0
  // POS fromPos, withPos;
#if 0
  trip.push_back(getDumpSite());
  Bucket inPath, notInPath;
  for (UINT i = 1; i < trip.size()-1; ++i) {
    if (twc->isInPath(trip[i-1], trip[i], trip[i+1])) {
      inPath.push_back(trip[i]);
    } else {
      notInPath.push_back(trip[i]);
    };
  }
  inPath.dumpid("inPath");
  notInPath.dumpid("notInPath");
#endif

  // double removeTime, insertTime, deltaTime, bestDelta;
  POS i_pos;
  double i_delta;
  for (int pos = 2; pos < size() - 1; ++pos) {
  // for (int pos = size() - 1; pos > 0; --pos) {
#if 0
    // DLOG(INFO) << "POS = " << pos;
    UINT from = trip[pos-1].nid();
    UINT middle = trip[pos].nid();
    UINT to = trip[pos+1].nid();
    Trashnode node = trip[pos];
    bestDelta = 9999999;
    int bestI=pos-1;

    removeTime = twc->TravelTime(from, to) - twc->TravelTime(from, middle, to);
    // DLOG(INFO) << trip[pos-1].id() << " " << node.id() << " " << trip[pos+1].id() << " from " << twc->TravelTime(from, middle, to);
    // DLOG(INFO) << trip[pos].id() << " " << trip[pos+1].id() << "to" << twc->TravelTime(from, to);
#endif
    // trip.tau("before erasing the last node");

    UINT d_pos = pos;
    double d_delta;
    auto d_node = path[d_pos].nid();
    //UINT d_node;
    // bestRemoval(d_node, d_pos, d_delta);
    auto node = path[d_pos];
    // e_remove(d_pos);
    trip.bestInsertion(d_node, i_pos, i_delta);
    trip.e_insert(node, i_pos);
    DLOG(INFO) << "node " << node.id() << " deleted from " << d_pos  << "\t inserted in " << i_pos << " after " << trip[i_pos - 1].id();
    trip.tau("after");
#if 0
    if (d_pos != i_pos) {
//      DLOG(INFO) << "node deleted from " << pos << " after " << path[pos - 1].id() << "\t inserted in " << i_pos << " after " << path[i_pos - 1].id();
      tau("after");
      // pos = 1;
    }
#endif


#if 0    
    // trip.tau("after erasing the last node");
    for (UINT i = 0; i < trip.size()-2; ++i) {
      UINT i_nid = trip[i].nid();
      UINT j_nid = trip[i+1].nid();

      insertTime = twc->TravelTime(i_nid, middle, j_nid) - twc->TravelTime(i_nid, j_nid);
      // DLOG(INFO) << trip[i].id() << " " << trip[i+1].id() << " from " << twc->TravelTime(i_nid, j_nid);
      // DLOG(INFO) << trip[i].id() << " " << node.id() << " " << trip[i+1].id() << "  to" << twc->TravelTime(i_nid, middle, j_nid);
      // DLOG(INFO) << "remove " <<  removeTime << " insert: " << insertTime << " delta: "<< removeTime+insertTime;
      deltaTime = removeTime + insertTime;
      if (deltaTime < bestDelta) {
        bestDelta = deltaTime;
        bestI = i;
      }
    }  // for i

    // DLOG(INFO) << "inserting after " << trip[bestI].id();
    trip.e_insert(node, bestI+1);
    // trip.tau("after moving the last node");
    if (bestI == pos+1) pos = trip.size()-1;
#endif

  }  // for pos

#endif
  osrmi->useOsrm(oldStateOsrm);
}





bool Vehicle::exchangesWithOnPath(Trip &trip, Trip &o_trip) {
  if (!(trip.trip_id() > o_trip.trip_id() )) return false;
  POS  d_pos, i_pos;
  POS  o_d_pos, o_i_pos;
  double d_delta, i_delta;
  double o_d_delta, o_i_delta;
  bool inPath1, inPath2;
  UINT d_node, o_d_node;
  
  Bucket nodesOnTripPath;
  trip.getNodesOnPath(o_trip, nodesOnTripPath);
  if (nodesOnTripPath.size() == 0) {
    DLOG(INFO) << "We cant  exchange: o_trip does not have nodes in trip's path" ;
    return false; 
  }
  assert(nodesOnTripPath.size() > 0);
  nodesOnTripPath.dumpid("nodes on path");
  while (nodesOnTripPath.size() > 0) { 
    o_d_node = nodesOnTripPath[0].nid();
    o_d_pos  = o_trip.Path().pos(o_d_node);
    o_d_delta = o_trip.delta_del(o_d_pos);
   

    // trip wants to give o_trip his worse node: the one when its removed decreases the time by a lot
    trip.bestRemoval(d_node, d_pos, d_delta);
    o_trip.bestInsertion(d_node, o_i_pos, o_i_delta);
    trip.bestInsertion(o_d_node, i_pos, i_delta);

    // condtitions to do the exchange
    DLOG(INFO) <<  "d_delta + o_i_delta " << d_delta << " +  " << o_i_delta  <<   " = " << d_delta + o_i_delta;
    DLOG(INFO) <<  "o_d_delta + i_delta " << o_d_delta << " +  " << i_delta  <<   " = " << o_d_delta + i_delta;
    DLOG(INFO) << " total del delta " << o_d_delta + d_delta << " total ins delta " <<  o_i_delta + i_delta << " total "  << o_d_delta + d_delta + o_i_delta + i_delta  ;

    DLOG(INFO) <<  "trip " << trip.trip_id() << "\tnode " << trip[d_pos].id() << " to o_trip " << o_trip.trip_id() << "\t after " << o_trip[o_i_pos - 1].id();
    DLOG(INFO) <<  "o_trip " << o_trip.trip_id() << "\tnode " << o_trip[o_d_pos].id() << " to trip " << trip.trip_id() << "\t after " << trip[i_pos - 1].id();
    if (o_d_delta + d_delta + o_i_delta + i_delta > 0) {
      // nodesOnTripPath.pop_front();
      // continue;
      DLOG(INFO) <<  " main " << trip.trip_id() << " delta: " << d_delta + i_delta;
      DLOG(INFO) <<  "o_trip " << o_trip.trip_id() << " delta: " << o_d_delta + o_i_delta;
      if ( d_delta + i_delta > 0) {
        nodesOnTripPath.pop_front();
        continue;
      }
    }

    trip.exchange(o_trip, d_pos, o_i_pos, o_d_pos, i_pos);
    nodesOnTripPath.pop_front();
  }
  return true;
}



int Vehicle::exchangesWorse(int lim_iter) {
  auto count = 0;
  for (auto &trip : trips) {
     if (trip.trip_id() == 0) continue;
     auto other  = trip.trip_id() - 1;
     DLOG(INFO) << "excahnging worse " << trip.trip_id() << " with " << other;
     count += exchangesWorse(trip, trips[other], lim_iter);
  }
  return count;
}

int Vehicle::exchangesWorse(Trip &trip, Trip &o_trip, int lim_iter) {
  for (UINT i = 0; i< lim_iter; ++i) {
    if (!exchangesWorse(trip, o_trip)) return i;
  }
  return lim_iter;
}


bool Vehicle::exchangesWorse(Trip &trip, Trip &o_trip) {
  POS  del_pos, ins_pos;
  POS  o_del_pos, o_ins_pos;
  double delta_del, delta_ins;
  double o_delta_del, o_delta_ins;
  bool inPath1, inPath2;

  trip.getRemovalValues(o_trip, o_ins_pos, del_pos, delta_del, o_delta_ins);
  o_trip.getRemovalValues(trip, ins_pos, o_del_pos, o_delta_del, delta_ins);
  if ( delta_del + delta_ins + o_delta_del + o_delta_ins <= 0) {
    if (trip[del_pos].id() == trip[ins_pos-1].id()
        ||  o_trip[o_del_pos].id() ==  o_trip[o_ins_pos-1].id()) return false;
     // DLOG(INFO) <<  "delta shortest " << delta_del + delta_ins ;
     DLOG(INFO) <<  "delta total " <<  delta_del + delta_ins + o_delta_del + o_delta_ins;
     DLOG(INFO) <<  "trip " << trip.trip_id() << "\tnode " << trip[del_pos].id() << " to trip " << o_trip.trip_id() << "\t after " << o_trip[o_ins_pos-1].id();
     DLOG(INFO) <<  "trip " << o_trip.trip_id() << "\tnode " << o_trip[o_del_pos].id() << " to trip " << trip.trip_id() << "\t after " << trip[ins_pos-1].id();
     trip.exchange(o_trip, del_pos, o_ins_pos, o_del_pos, ins_pos);
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
bool Trip::getRemovalValues(const Trip &other, POS &o_ins_pos, POS &del_pos, double &o_delta_ins, double &delta_del) const{
  UINT del_node;
  UINT ins_after;
  bool insertInPath;
  bestRemoval(del_node, del_pos, delta_del);
  return other.bestInsertion(del_node, o_ins_pos, o_delta_ins);
  
//  tau("best removal trip:");
  DLOG(INFO) << "\tnode to be removed: " << path[del_pos].id()
             << " at pos: " << del_pos
             << "\tdelta: " << delta_del;
  DLOG(INFO) << "\tinsert after node: " << other[o_ins_pos-1].id()
             << " at pos: " << o_ins_pos
             << "\tdelta: " << o_delta_ins;
  double delta = o_delta_ins + delta_del;
  return delta < 0;
};


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

void Trip::getNodesOnPath(const Trip &o_trip, POS o_i_pos, Bucket &nodesOnPath) const {
  Bucket o_nodes;
  nodesOnPath.clear();
  o_nodes = o_trip.path; // choose from this
  if (o_i_pos - 1 < o_nodes.size()) o_nodes.erase(o_i_pos - 1); // this nodes the other cant give
  if (o_i_pos - 1 < o_nodes.size()) o_nodes.erase(o_i_pos - 1);
  o_nodes.pop_front();  // delete the starting site
  twc->getNodesOnPath(path, dumpSite, o_nodes, nodesOnPath);
};

///////////////////////////////////////

void Trip::orderNodesAlongPath(Bucket &orderedNodes) const {
  orderedNodes.clear();
  twc->orderNodesAlongPath(path, dumpSite, orderedNodes);
  orderedNodes.pop_back();  // delete the dumpSite
};

void Trip::orderNodesAlongPath() {
  Bucket orderedNodes;
  orderNodesAlongPath(orderedNodes);
#ifdef VRPMINTRACE
  if (path.size() != orderedNodes.size()) {
    DLOG(INFO) << "path.size: " << path.size();
    DLOG(INFO) << "orderedNodes.size: " << orderedNodes.size();
    path.dump("path");
    orderedNodes.dump("orderedNodes");
  }
  assert( path.size() == orderedNodes.size() );
#endif
  path = orderedNodes;
};

///////////////////////////////////////

void Trip::getNodesOnPath(const Trip &o_trip, Bucket &nodesOnPath) const {
  Bucket o_nodes;
  nodesOnPath.clear();
  o_nodes = o_trip.path; // choose from this
  o_nodes.pop_front();  // delete the starting site
  twc->getNodesOnPath(path, dumpSite, o_nodes, nodesOnPath);
};

bool Trip::chooseMyBest(const Trip &other, POS o_ins_pos, POS del_pos, POS &ins_pos, POS &o_del_pos, double &o_delta_del, double &i_delta) const {
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
  removeRestricted(nodesOnPath, del_pos);
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
        bestInsertion(o_del_node, ins_pos, i_delta);
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

double Trip::delta_del(POS del_pos) const {
  assert(del_pos > 0 && del_pos < path.size());
  double time0, time1;
  if (del_pos == path.size()-1) { // its the last node
    //TODO quitar el .nid();
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


double Trip::delta_ins(UINT n_ins, POS ins_pos) const {
  assert(ins_pos > 0 && ins_pos <= path.size());
  double time0, time1;
    if (ins_pos == path.size()) {
      if (twc->isInPath(path[ins_pos - 1].nid(), n_ins, dumpSite.nid())) return 0;
      time1 = twc->TravelTime(path[ins_pos - 1].nid(), n_ins, dumpSite.nid());
      time0 = twc->TravelTime(path[ins_pos - 1].nid(), dumpSite.nid());
    } else {
      if (twc->isInPath(path[ins_pos - 1].nid(), n_ins, path[ins_pos].nid()))  return 0;
      time1 = twc->TravelTime(path[ins_pos - 1].nid(), n_ins, path[ins_pos].nid());
      time0 = twc->TravelTime(path[ins_pos - 1].nid(), path[ins_pos].nid());
    }
    return time1 - time0;
}





#if 0
bool Trip::ord_delta_del(const Trashnode &n1, const Trashnode &n2) const {
  return delta_del(path.pos(n1.nid())) <  delta_del(path.pos(n2.nid()));
}


void Trip::sortBasedOnBestRemoval(Bucket &nodes) const {
    std::sort( nodes.begin(), nodes.end(), [=] {
       return delta_del(path.pos(n1.nid())) <  delta_del(path.pos(n2.nid()))
    };
}
#endif

// return the deltaTime
void Trip::bestRemoval(UINT &d_node, POS &d_pos, double &d_delta) const {
  assert(path.size() > 1); // if size == 1 the trip shoudnt exist
  d_pos = 1;
  if (size()==1) {
    d_node = path[1].nid();
    d_delta = delta_del(1);
    return;
  }
  double deltaTime;
  d_delta = 99999;
  for (UINT i = 1; i < path.size(); ++i) {
    deltaTime = delta_del(i); 

    if (deltaTime < d_delta) {
      d_delta = deltaTime;
      d_pos = i;
      d_node = path[d_pos].nid();
    }
  }
  return;
}

bool Trip::bestInsertion(UINT n_ins, POS &ins_pos, double &i_delta) const {
  assert(path.size() > 1); // will never use insert into an empty truck
  ins_pos = 1;
  i_delta = 999999;

  double time0, time1, deltaTime;
  for (POS i = 1; i <= path.size(); ++i) {
    deltaTime = delta_ins(n_ins, i);
    // DLOG(INFO) << i << "delta" << deltaTime;
    if (deltaTime < i_delta) {
      i_delta = deltaTime;
      ins_pos = i;
      //DLOG(INFO) << i << "delta" << i_delta;
      if (i_delta <= 0 ) return true; // is on the path or inserting decreses the time?
    }
  }
  return false;
}


//
void Trip::removeRestricted(Bucket &nodesOnPath, POS special) const {
  assert(path.size() > 1); // will never use insert into an empty truck
  UINT ins_pos;
  double i_delta;
  Bucket ok;
  for (POS i = 0; i <= nodesOnPath.size(); ++i) {
    bestInsertion(nodesOnPath[i].nid(), ins_pos, i_delta);
    if (ins_pos != special && ins_pos != (special + 1)) 
      ok.push_back(nodesOnPath[i]);
  }
  nodesOnPath = ok;
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
  for (const auto &trip : trips) {
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
    
void Vehicle::reconstruct() {
    auto tmp_trips = trips;
    trips.clear();
    for (Trip &trip : tmp_trips) {
      add_trip(trip);
    }
};


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
