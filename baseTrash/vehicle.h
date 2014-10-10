#ifndef VEHICLE_H
#define VEHICLE_H

#include <limits>
#include <vector>
#include <sstream>


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "plot.h"
#include "move.h"
#include "basevehicle.h"


class Vehicle: public BaseVehicle {
  private:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID ;
    inline double _MAX() const { ( std::numeric_limits<double>::max() ); };
    inline double _MIN() const { ( - std::numeric_limits<double>::max() ); };

/*
*/
  protected:

  public:
    // TODO LIST 
    // insertion will not be performed 
    //  return false if TV or CV is generated 
    bool eval_insertSteadyDumps(const Trashnode &node, int at) const;

    bool e_insertIntoFeasableTruck(const Trashnode &node,int pos);
    // insertion will be performed and return false if TV or CV is generated 
    bool e_insertMoveDumps(const Trashnode &node, int at);
    bool e_insertSteadyDumps(const Trashnode &node, int at);
    bool e_insert(const Trashnode &node, int at) { return  e_insertMoveDumps(node, at); };


    // Very TIGHT insertion 
    // insertion will not be performed if 
    //      TV and CV are  generated 
    //  true- insertion was done
    //  false- not inserted 
    bool e_insertMoveDumpsTight(const Trashnode &node, int at);
    bool e_insertSteadyDumpsTight(const Trashnode &node, int at);
    bool e_insertTight(const Trashnode &node, int at) { return  e_insertMoveDumpsTight(node, at); };
    // END TODO LIST
    
    bool eval_erase(int at, double &savings) const;
    bool applyMoveINSerasePart(int nodeNid, int pos);
    bool applyMoveINSinsertPart(const Trashnode &node, int pos);
    bool e_makeFeasable(int currentPos);
    long int eval_insertMoveDumps( const Trashnode &node, std::deque<Move> &moves, int fromTruck, int formPos, int toTruck, double savings, double factor ) const;
    long int eval_interSwapMoveDumps( std::deque<Move> &moves, const Vehicle &otherTruck,int  truckPos,int  otherTruckPos, int fromPos,  double factor) const;
    bool e_insertDumpInPath( const Trashnode &going );
    bool deltaTimeGeneratesTV(const Trashnode &dump, const Trashnode &node) const; 
    bool deltaCargoGeneratesCV(const Trashnode &node, int pos) const;
    bool deltaCargoGeneratesCV_AUTO(const Trashnode &node, int pos) const;
    bool deltaTimeGeneratesTV(const Trashnode &node,int pos) const;
    bool deltaTimeGeneratesTV_AUTO(const Trashnode &node,int pos) const;
    //--------------------------------------------------------------------
    // constructors
    //--------------------------------------------------------------------

    Vehicle() {
        maxcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
    };

    Vehicle(std::string line,const Bucket &otherlocs): BaseVehicle(line,otherlocs)   { }


};


#endif

