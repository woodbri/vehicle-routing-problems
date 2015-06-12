#ifndef TRIPVEHICLE_H
#define TRIPVEHICLE_H

#include "twc.h"
#include "vehicle.h"


// TODO move the code to .cpp keep the headers

class Trip: public Vehicle1 {
  public:
  Trip():Vehicle1(){};
  int m_trip_id;

  Trip(const Trashnode &S, const Trashnode &D, const Trashnode &E, double maxcap)
    : Vehicle1(S, D, E, maxcap), m_trip_id(0) { };


  void swapBestToDump(Trip &other);
  void exchange(Trip &other, POS del_pos, POS o_ins_pos, POS o_del_pos, POS ins_pos);
  void intraTripOptimizationNoOsrm();
  void getNodesInOrder();

  

  bool operator < (const Trip &o_trip) const;
  void getNodesOnPath(const Trip &o_trip, POS o_ins_pos, Bucket &nodesOnPath) const;
  void getNodesOnPath(const Trip &o_trip, Bucket &nodesOnPath) const;
  void getNodesNotOnPath(const Trip &o_trip, Bucket &nodesNotOnPath) const;
  double  delta_del(POS del_pos) const;
  double  delta_ins(UINT n_nid, POS del_pos) const;
  int&   trip_id() {return m_trip_id;} 
  void removeRestricted(Bucket &nodesOnPath, POS special) const;
  bool getRemovalValues(const Trip &other, POS &o_ins_pos, POS &del_pos, double &o_delta_ins, double &delta_del) const;
  bool chooseMyBest(const Trip &other,  POS o_ins_pos, POS del_pos,  POS &ins_pos, POS &o_del_pos, double &o_delta_ins, double &delta_ins) const;
  void bestRemoval(UINT &d_node, POS &d_pos, double &d_delta) const;
  bool bestInsertion(UINT n_id, POS &ins_pos, double &delta_ins) const;
};


class Vehicle: public Vehicle1 {
 public:
  std::deque< Trip > trips;


  Vehicle():Vehicle1(){};
  Vehicle(const std::string &line, const Bucket &otherlocs )
    : Vehicle1(line, otherlocs)   { };
  Vehicle( int _vid, int _start_id, int _dump_id, int _end_id,
           int _capacity, int _dumpservicetime, int _starttime,
           int _endtime, const Bucket &otherlocs )
    : Vehicle1( _vid, _start_id, _dump_id, _end_id,
                   _capacity, _dumpservicetime, _starttime,
                   _endtime, otherlocs ) {};



  void basicOptimization();
  ///{@
  int  exchangesWorse(int lim_iter);
  int  exchangesWorse(Trip &trip, Trip &o_trip, int lim_iter);
  bool exchangesWorse(Trip &trip, Trip &o_trip);
  ///@}

  int exchangesWithOnPath(Trip &trip, Trip &o_trip);
  int exchangesWithNotOnPath(Trip &trip, Trip &o_trip);

  void swapBestToDump();
  void intraTripOptimizationNoOsrm();

  void tauTrips() const;
  void tauTrips(const std::string &title) const;
  Trip get_new_trip();
  void reconstruct();
  void add_trip(const Trip &p_trip);
};
#endif // TRIPVEHICLE_H
