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


    Bucket original;
    std::vector<std::vector<double> > twcij;
    std::vector<std::vector<double> > distance;

    inline double _MIN() const { return -std::numeric_limits<double>::max();};
    inline double _MAX() const { return std::numeric_limits<double>::max();};




public:


bool isCompatibleIJ(UID fromNid, UID toNid) const {
    assert(fromNid<original.size() and toNid<original.size() );
    return not (twcij[fromNid][toNid]  == _MIN());
}

bool isCompatibleIAJ(UID fromNid, UID middleNid, UID toNid) const {
    assert(fromNid<original.size() and middleNid<original.size()  and toNid<original.size() );
    return isCompatibleIJ(fromNid,middleNid) and isCompatibleIJ(middleNid,toNid);
}



double compatibleIJ(UID fromNid, UID toNid) const {
    assert(fromNid<original.size() and toNid<original.size() );
    return  twcij[fromNid][toNid] ;
}



/*compatibility hast to be nodeid based not position based*/
const knode& node(UID i) const {
    assert(i<original.size() );
    return original[i];
};

const knode& getNode(UID at) const {
    assert(at<original.size() );
    return original[at];
};



int getSeed(int foo, const Bucket &nodes) {
     int bestId,count;
     double bestEc2;
     int Id;
     int bestCount=0;
     bestEc2= - _MIN();
//std::cout<<"gettingSeed";
     for (int i=0; i<nodes.size(); i++) {
         if (i==0) bestId = nodes[0].getnid();
         Id = nodes[i].getnid();
//std::cout<<"\n working with node "<<Id;
         count=0;
        for (int j=0; j<nodes.size(); j++)
            if ( i!=j and  isCompatibleIJ( Id , nodes[j].getnid() ) ) count++;
//std::cout<<" has  "<<ec2(Id)<<"ec2";
//std::cout<<" has  "<<count<<" compatibilities";
        if (count>bestCount) {
//std::cout<<" oldCount  "<<bestCount<<" newCount "<<count;
            bestCount=count;
            bestId=Id;
         }
     }
     return bestId;
}



//  only in current nodes 
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
    assert( nodes.size() );
    std::cout<<"DUMPINGGGGG \n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"pos "<<i<<"\t";

    std::cout<<"\n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"nid "<<nodes[i].getnid()<<"\t";

    std::cout<<"\n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"id "<<nodes[i].getid()<<"\t";

    std::cout<<"\n";
    for (int i=0;i<nodes.size();i++){
        std::cout<<nodes[i].getnid()<<"\t";
        for (int j=0; j<nodes.size();j++) {
           if (twcij[i][j] !=  -std::numeric_limits<double>::max()) std::cout<<twcij[i][j]<<"\t";
           else std::cout<<"--\t";
        }
        std::cout<<"\n";
    }
}


void dumpCompatible() const  {
    assert( original.size() );
    dumpCompatible(original);
} 

void dumpCompatible(const Bucket &nodes) const {
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



// Functions to adjust compatability depending on problem 

//go back to CALCULATED state
void recreateCompatible( UID nid) {
     assert(nid<original.size() );
     for (int j=0; j<twcij.size(); j++) {
         twcij[nid][j]= twc_for_ij(original[nid],original[j]);
         twcij[j][nid]= twc_for_ij(original[j],original[nid]);
     }
}

void recreateiDistance( UID nid) {
     assert(nid<original.size() );
     for (int j=0; j<twcij.size(); j++) {
         distance[nid][j]=  distance[j][nid]= original[j].distnace(original[nid]);
     }
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
    distance[fromNid][toNid]= _MAX();
}

void setUnreachable(UID nid, const Bucket &nodes) {
     assert(nid<original.size() );
     for (int j=0; j<nodes.size(); j++)
         distance[nid][nodes[j].getnid()]=  _MAX();
}

void setUnreachable(const Bucket &nodes, UID &nid ) {
     assert(nid<original.size() );
     for (int i=0; i<nodes.size(); i++)
         distance[nodes[i].getnid()][nid]=  _MAX();
}






int setNodes(Bucket _original) {
    original.clear();
    original=_original;
    twcij_calculate();
    assert (original==_original);
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
    return ni.closes()+ni.getservicetime()+nj.distance(ni);
}

double ajei(const knode &ni, const knode &nj) {
    return ni.opens()+ni.getservicetime()+nj.distance(ni);
}


double twc_for_ij(const knode &ni, const knode &nj) {
    double result;
    if ( ( nj.closes() -ajei(ni,nj) ) > 0 ) {
        result = std::min ( ajli(ni,nj) , nj.closes() )
                  - std::max ( ajei(ni,nj) , nj.opens()  ) ;
    } else result= _MIN();
    return result;
}



/* public functions That are id based */
void twcij_calculate(){
    assert (original.size());
    twcij.resize(original.size());
    distance.resize(original.size());

    for (int i=0;i<original.size();i++) {
        twcij[i].resize(original.size());
        distance[i].resize(original.size());
    }

    for (int i=0;i<original.size();i++){
        for (int j=i; j<original.size();j++) {
              distance[i][j]= distance[j][i]= original[j].distance(original[i]);
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
