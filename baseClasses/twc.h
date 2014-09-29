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
#include <limits>

#include "node.h"
#include "twpath.h"


template <class knode> class TWC {
private:
typedef TwBucket<knode> Bucket;
typedef unsigned long int UID;


    TwBucket<knode> original;
    std::vector<std::vector<double> > twcij;
    std::vector<std::vector<double> > travel_Time;

    inline double _MIN() const { return -std::numeric_limits<double>::max();};
    inline double _MAX() const { return std::numeric_limits<double>::max();};




public:

// major tools
bool  findNearestNodeTo(const TwBucket<knode> &truck, const  TwBucket<knode> &unassigned,  UID &pos, knode &bestNode, double &bestDist) const {
    assert( unassigned.size() );
    int flag=false;
    bestDist = _MAX();   // dist to minimize
    pos = 0;        // position in path to insert
    double d;


    for (int i=0; i<unassigned.size(); i++) {
        for (int j=0; j<truck.size()-1; j++) {
           if ( isCompatibleIAJ(truck[j],unassigned[i],truck[j+1])) {
              d = truck.segmentDistanceToPoint( j , unassigned[i] );
              if ( d < bestDist ) {
                bestDist = d;
                pos = j+1;
                bestNode = unassigned[i];
                flag=true;
              }
           }
        }
    }

    return flag;
}




TwBucket<knode> getUnreachable(const TwBucket<knode> &nodes, UID to) const {
    assert( to<original.size() );
    Bucket unreachable;
    for (int i=0; i<nodes.size(); i++)
        if (not isReachableIJ(nodes[i].getnid(),to))
           unreachable.push_back(nodes[i]);
    return unreachable;
}

TwBucket<knode> getUnreachable(UID from, const TwBucket<knode> &nodes) const {
    assert( from<original.size() );
    Bucket unreachable;
    for (int i=0; i<nodes.size(); i++)
        if (not isReachableIJ(from,nodes[i].getnid()))
           unreachable.push_back(nodes[i]);
    return unreachable;
}

TwBucket<knode> getReachable(const TwBucket<knode> &nodes, UID to) const {
    assert( to<original.size() );
    Bucket reachable;
    for (int i=0; i<nodes.size(); i++)
        if (isReachableIJ(nodes[i].getnid(),to))
           reachable.push_back(nodes[i]);
    return reachable;
}

TwBucket<knode> getReachable(UID from, const TwBucket<knode> &nodes) const {
    assert( from<original.size() );
    Bucket reachable;
    for (int i=0; i<nodes.size(); i++)
        if (iReachableIJ(from,nodes[i].getnid()))
           reachable.push_back(nodes[i]);
    return reachable;
}


TwBucket<knode> getIncompatible(const TwBucket<knode> &nodes, UID to) const {
    assert( to<original.size() );
    Bucket incompatible;
    for (int i=0; i<nodes.size(); i++) 
        if (not isCompatibleIJ(nodes[i].getnid(),to)) 
           incompatible.push_back(nodes[i]);
    return incompatible;
}
    
TwBucket<knode> getIncompatible(UID from, const TwBucket<knode> &nodes) const {
    assert( from<original.size() );
    Bucket incompatible;
    for (int i=0; i<nodes.size(); i++) 
        if (not isCompatibleIJ(from,nodes[i].getnid())) 
           incompatible.push_back(nodes[i]);
    return incompatible;
}

TwBucket<knode> getCompatible(const TwBucket<knode> &nodes, UID to) const {
    assert( to<original.size() );
    Bucket compatible;
    for (int i=0; i<nodes.size(); i++) 
        if (isCompatibleIJ(nodes[i].getnid(),to))
           compatible.push_back(nodes[i]);
    return compatible;
}
    
TwBucket<knode> getCompatible(UID from, const TwBucket<knode> &nodes) const {
    assert( from<original.size() );
    Bucket compatible;
    for (int i=0; i<nodes.size(); i++) 
        if (isCompatibleIJ(from,nodes[i].getnid())) 
           compatible.push_back(nodes[i]);
    return compatible;
}
    

double travelTime(UID from, UID to) const {
    assert(from<original.size() and to<original.size() );
    return travel_Time[from][to];
}
 
double travelTime(const knode &from, const knode &to) {
    return travelTime(from.getnid(),to.getnid());
}

double compatibleIJ(UID fromNid, UID toNid) const {
    assert(fromNid<original.size() and toNid<original.size() );
    return  twcij[fromNid][toNid] ;
}

const knode& node(UID i) const {
    assert(i<original.size() );
    return original[i];
};

const knode& getNode(UID at) const {
    assert(at<original.size() );
    return original[at];
};

//state
bool isCompatibleIJ(UID fromNid, UID toNid) const {
    assert(fromNid<original.size() and toNid<original.size() );
    return not (twcij[fromNid][toNid]  == _MIN());
}

bool isiReachableIJ(UID fromNid, UID toNid) const {
    assert(fromNid<original.size() and toNid<original.size() );
    return not (travel_Time[fromNid][toNid]  == _MIN());
}


bool isCompatibleIAJ(UID fromNid, UID middleNid, UID toNid) const {
    assert(fromNid<original.size() and middleNid<original.size()  and toNid<original.size() );
    return isCompatibleIJ(fromNid,middleNid) and isCompatibleIJ(middleNid,toNid);
}


bool isCompatibleIAJ(const knode &from, const knode &middle, const knode &to) const {
    return isCompatibleIAJ( from.getnid(), middle.getnid() ,to.getnid() );
}






//  The best or the worses  

knode findBestTravelTime(UID from, const Bucket &nodes) const {
    assert (nodes.size() and from <original.size());
    Bucket reachable=getReachable(from,nodes);
    if (not reachable.size()) return nodes[0];
    knode best = reachable[0];
    double bestTime = _MAX();  
    for (int i=0; i<reachable.size(); i++) {
        if (reachable[i].getnid()!=from and travelTime(from,reachable[i].getid()) < bestTime) {
            best=reachable[i];
            bestTime=travelTime(from,reachable[i].getid());
        }
    }
    return best;
}

knode findBestTravelTime(const Bucket &nodes, UID to) const {
    assert (nodes.size() and to<original.size());
    Bucket reachable=getReachable(nodes,to);
    if (not reachable.size()) return nodes[0];
    knode best = reachable[0];
    double bestTime = _MAX(); 
    for (int i=0; i<reachable.size(); i++) {
        if (reachable[i].getnid()!=to and travelTime(reachable[i].getid(),to) < bestTime) {
            best=reachable[i];
            bestTime=travelTime(reachable[i].getid(),to);
        }
    }
    return best;
}

knode findWorseTravelTime(UID from, const Bucket &nodes) const {
// from the reachable nodes finds the worse
    assert (nodes.size() and from <original.size());
    Bucket reachable=getReachable(from,nodes);
    if (not reachable.size()) return nodes[0];
    knode worse = reachable[0];
    double worseTime = _MIN(); 
    for (int i=0; i<reachable.size(); i++) {
        if (reachable[i].getnid()!=from and travelTime(from,reachable[i].getid()) > worseTime) {
            worse=reachable[i];
            worseTime=travelTime(from,reachable[i].getid());
        }
    }
    return worse;
}

knode findWorseTravelTime(const Bucket &nodes, UID to) const {
// from the reachable nodes finds the worse
    assert (nodes.size() and to <original.size());
    Bucket reachable=getReachable(nodes,to);
    if (not reachable.size()) return nodes[0];
    knode worse = reachable[0];
    double worseTime = _MIN();
    for (int i=0; i<reachable.size(); i++) {
        if (reachable[i].getnid()!=to and travelTime(reachable[i].getid(),to) > worseTime) {
            worse=reachable[i];
            worseTime=travelTime(reachable[i].getid(),to);
        }
    }
    return worse;
}



int getSeed(int foo, const Bucket &nodes) const {
//ec2 get seed (needs revision)
     int bestId,count;
     double bestEc2;
     int Id;
     int bestCount=0;
     bestEc2= - _MIN();
     for (int i=0; i<nodes.size(); i++) {
         if (i==0) bestId = nodes[0].getnid();
         Id = nodes[i].getnid();
         count=0;
        for (int j=0; j<nodes.size(); j++) {
            if ( i!=j and  isCompatibleIJ( Id , nodes[j].getnid() ) ) count++;
            bestCount=count;
            bestId=Id;
         }
     }
     return bestId;
}



int  getBestCompatible(UID fromNid, const Bucket &nodes) const {
     assert(fromNid<original.size() );
     int bestId;
     int toId;

     if (nodes.empty()) return -1;
     bestId=nodes[0].getnid();

     for (int j=0; j<nodes.size(); j++) {
         toId = nodes[j].getnid();
         if ( twcij[fromNid][toId] > twcij[fromNid][bestId]) {
                 bestId=toId;
         }
     }
     if (compat(fromNid,bestId)!=_MIN()) return bestId;
     else return -1;
}




double ec2(UID at, const Bucket &nodes) {
     assert(at<original.size() );
     double ec2_tot=0;
     for (int j=0; j<nodes.size(); j++) {
         if ( not (twcij[at][j]  == _MIN()) ) ec2_tot+=twcij[at][j];
         if ( not (twcij[j][at]  == _MIN()) ) ec2_tot+=twcij[j][at];
     };
     if ( twcij[at][at] == _MIN() ) ec2_tot-=twcij[at][at];
     return ec2_tot;
}


//counting
int countIncompatibleFrom(int at, const Bucket &nodes) {
     assert(at<original.size() );
     int count=0;
     for (int j=0; j<nodes.size(); j++) {
         if ( twcij[at][j]  == _MIN() ) count++;
     }
     return count;
}

int countIncompatibleTo(int at, const Bucket &nodes) {
     int count=0;
     for (int j=0; j<nodes.size(); j++) {
         if ( twcij[j][at]  == _MIN() ) count++;
     }
     return count;
}



/*    DUMPS   */
void dump() const  {
    assert( original.size() );
    dump(original);
} 


void dump(const Bucket &nodes) const  {
    assert( original.size() );
    dumpCompatability(original);
    dumpTravelTime(original);
}

void dumpCompatability() const  {
    assert( original.size() );
    dumpCompatability(original);
}

void dumpCompatability(const Bucket &nodes) const  {
    assert( nodes.size() );
    std::cout.precision(8);
    std::cout<<"COMPATABILITY TABLE \n\t";

    for (int i=0;i<nodes.size();i++)
        std::cout<<"nid "<<nodes[i].getnid()<<"\t";

    std::cout<<"\n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"id "<<nodes[i].getid()<<"\t";

    std::cout<<"\n";
    for (int i=0;i<nodes.size();i++){
        std::cout<<nodes[i].getnid()<<"="<<nodes[i].getid()<<"\t";
        for (int j=0; j<nodes.size();j++) {
           if (twcij[i][j] !=  -std::numeric_limits<double>::max()) std::cout<<twcij[i][j]<<"\t";
           else std::cout<<"--\t";
        }
        std::cout<<"\n";
    }
}

void dumpTravelTime() const {
    assert( original.size() );
    dumpTravelTime(original);
}

void dumpTravelTime(const Bucket &nodes) const {
    assert( nodes.size() );
    std::cout<<"\n\n\n\nTRAVEL TIME TABLE \n\t";

    std::cout.precision(2);
    for (int i=0;i<nodes.size();i++)
        std::cout<<"nid "<<nodes[i].getnid()<<"\t";

    std::cout<<"\n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"id "<<nodes[i].getid()<<"\t";

    std::cout<<"\n";
    for (int i=0;i<nodes.size();i++){
        std::cout<<nodes[i].getnid()<<"="<<nodes[i].getid()<<"\t";
        for (int j=0; j<nodes.size();j++) {
           if (travelTime(i,j) !=  std::numeric_limits<double>::max()) std::cout<<travelTime(i,j)<<"\t";
           else std::cout<<"--\t";
        }
        std::cout<<"\n";
    }
}


void dumpCompatible3() const  {
    assert( original.size() );
    dumpCompatible(original);
} 

void dumpCompatible3(const Bucket &nodes) const {
    assert( nodes.size() );
    for (int i=0;i<nodes.size();i++) {
      for (int j=0;j<nodes.size();j++) {
        for (int k=0;k<nodes.size();k++) {
          std::cout<<"\t ( "<<nodes[i].getnid()<<" , "<<nodes[j].getnid()<<" , "<<nodes[k].getnid()<<") = "<<(isCompatibleIAJ(i,j,k)? "COMP": "not");
        }
        std::cout<<"\n";
      }
    }
}




//go back to CALCULATED state
void recreateCompatible( UID nid) {
     assert(nid<original.size() );
     for (int j=0; j<twcij.size(); j++) {
         twcij[nid][j]= twc_for_ij(original[nid],original[j]);
         twcij[j][nid]= twc_for_ij(original[j],original[nid]);
     }
}

// Functions to adjust compatability depending on problem 
void recreateTravelTime( UID nid) {
     assert("needs to be re-read from file"=="");
}


void setIncompatible(UID fromNid,UID toNid) {
    assert(fromNid<original.size() and toNid<original.size());
    twcij[fromNid][toNid]= _MIN();
}




void setIncompatible(UID nid, const Bucket &nodes) {
     assert(nid<original.size() );
     for (int j=0; j<nodes.size(); j++)
         twcij[nid][nodes[j].getnid()]=  _MIN();
}


void setIncompatible(const Bucket &nodes, UID &nid ) {
     assert(nid<original.size() );
     for (int i=0; i<nodes.size(); i++)
         twcij[nodes[i].getnid()][nid]=  _MIN();
}

void setUnreachable(UID fromNid,UID toNid) {
    assert(fromNid<original.size() and toNid<original.size());
    travel_Time[fromNid][toNid]= _MAX();
}

void setUnreachable(UID nid, const Bucket &nodes) {
     assert(nid<original.size() );
     for (int j=0; j<nodes.size(); j++)
         travel_Time[nid][nodes[j].getnid()]=  _MAX();
}

void setUnreachable(const Bucket &nodes, UID &nid ) {
     assert(nid<original.size() );
     for (int i=0; i<nodes.size(); i++)
         travel_Time[nodes[i].getnid()][nid]=  _MAX();
}






int setNodes(Bucket _original) {
    original.clear();
    original=_original;
    twcij_calculate();
    assert (original==_original);
    assert (check_integrity());
}
     
void loadAndProcess_distance(std::string infile, const Bucket &datanodes, const Bucket &invalid ) {
    assert(datanodes.size());
    original.clear();
    original=datanodes;

    std::ifstream in( infile.c_str() );
    std::string line;
    int fromId;
    int toId;

    travel_Time.resize(original.size());
    for (int i=0;i<original.size();i++)
        travel_Time[i].resize(original.size());
    //travel_Time default value is 250m/min
    for (int i=0;i<original.size();i++)
       for (int j=i;j<original.size();j++) 
          if (i==j) travel_Time[i][j]=0;
          else travel_Time[i][j]=travel_Time[j][i]=original[i].distance(original[j])*(1/250);

    int from,to;
    double dist;
    int cnt = 0;
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;
        std::istringstream buffer( line );
        buffer >> from;
        buffer >> to;
        buffer >> dist;
        if ( invalid.hasid(from) or invalid.hasid(to) ) continue;

        fromId=original.getNidFromId(from);
        toId=original.getNidFromId(to);
        travel_Time[fromId][toId]=dist;
    }
    in.close();

    twcij_calculate();
    assert (original==datanodes);
    assert (check_integrity());
}
 





// constructors
TWC() {};
TWC(Bucket _original)  : original(_original) {
    twcij_calculate();
    assert (original==_original);
    assert (check_integrity());
}

/* private are indexed */
private:
double compat(int i,int j) const {
    assert(i<original.size() and j<original.size());
    return twcij[i][j];
};

double ajli(const knode &ni, const knode &nj) {
    return ni.closes()+ni.getservicetime()+travelTime(ni,nj);
}

double ajei(const knode &ni, const knode &nj) {
    return ni.opens()+ni.getservicetime()+travelTime(ni,nj);
}


double twc_for_ij(const knode &ni, const knode &nj) {
    double result;
    if (travelTime(ni,nj)==_MAX()) result=_MIN();
    else if ( ( nj.closes() -ajei(ni,nj) ) > 0 ) {
        result = std::min ( ajli(ni,nj) , nj.closes() )
                  - std::max ( ajei(ni,nj) , nj.opens()  ) ;
    } else result= _MIN();
    return result;
}



/* public functions That are id based */
void twcij_calculate(){
    assert (original.size()==travel_Time.size());
    twcij.resize(original.size());

    for (int i=0;i<original.size();i++) 
        twcij[i].resize(original.size());
    

    for (int i=0;i<original.size();i++){
        for (int j=i; j<original.size();j++) {
              twcij[i][j]= twc_for_ij(original[i],original[j]);
              twcij[j][i]= twc_for_ij(original[j],original[i]);
        }
    }
}

bool check_integrity() const {
    assert (original.size()==twcij.size());
    for (int i=0; i<original.size();i++) {
        assert (twcij[i].size()==original.size());
    }
    return true;
}



};


#endif
