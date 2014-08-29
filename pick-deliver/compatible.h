#ifndef COMPATIBLE_H
#define COMPATIBLE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <math.h>

#include "node.h"
#include "order.h"
#include "vehicle.h"

class Compatible {
  protected:

    Twpath<Dpnode> nodes;
    Vehicle original;

    std::deque<Vehicle> subset;

    std::map<int,int> IdPos;
    std::deque<std::deque<double> > twcij;



/* not working for several depots */
    std::deque<double>  twc0;

    double ajli(const Dpnode &ni, const Dpnode &nj);
    double ajei(const Dpnode &ni, const Dpnode &nj);
    double twc_for_ij(const Dpnode &ni, const Dpnode &nj);

  public:
    Compatible() {};
    Compatible(Vehicle _original);
    int setSubset(Vehicle _subset);
    int setNodes(Vehicle _original);
  

    //void setTwcij(const Order& order);
        
    void setIncompatible(const Order& order);
    void setIncompatible(int fromId, int toId);
    void twcij_calculate();
    void twc_from_depot_calculate();
    double compat(int i,int j) const ;
    bool compatibleIJ(int i, int j);
    bool compatibleIAJ(int i, int a, int j);
    void dumpCompatible() ;
    void maskHorizontal(int at) ;
    void maskVertical(int at) ;
    int  getBestCompatible(int from) ;
    int  getBestPickupCompatible(int from) ;
    int  getBestCompatible() ;
    int  getBestPickupCompatible() ;
    void recreateRowColumn( int nodeId );
    double ec2(int nodeId);
    int countIncompatibleFrom(int nodeId);
    int countIncompatibleTo(int nodeId);
    int getIdOfWorseCount(int subsetId) ;




    void dump() const;

};

#endif
