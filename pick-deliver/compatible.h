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
    double compat(int i,int j) const ;

  public:
    Compatible() {};
    Compatible(Vehicle _original);
    int setSubset(Vehicle _subset);
    int setNodes(Vehicle _original);
  
    Dpnode getNode(int nodeId);

    //void setTwcij(const Order& order);
        
    void setIncompatible(const Order& order);
    void setIncompatible(int fromId, int toId);
    void twcij_calculate();
    void twc_from_depot_calculate();
    bool isCompatibleIJ(int fromId, int toId) ;
    bool isCompatibleIAJ(int fromId, int middleId, int toId) ;
    double compatibleIJ(int fromId,int toId) ;
    void dumpCompatible() ;
    void maskHorizontal(int at) ;
    void maskVertical(int at) ;
    int  getBestCompatible(int from) ;
    int  getSeed(int fromNid, const Bucket &nodes);
    int  getBestCompatible(int fromNid,const Bucket &nodes);
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
