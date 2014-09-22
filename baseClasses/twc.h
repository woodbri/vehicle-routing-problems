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
//#include "order.h"
//#include "bucketn.h"



template <class knode> class TWC {
  protected:
typedef TwBucket<knode> Bucket;


    TwBucket<knode> nodescitos;
    Bucket original;


    std::deque<std::deque<double> > twcij;



    inline double _MIN() const { return -std::numeric_limits<double>::max();};




public:

int setNodes(Bucket _original) {
    original=_original;
    twcij_calculate();
}






bool isCompatibleIJ(int fromNid, int toNid) const {
    return not (twcij[fromNid][toNid]  == _MIN());
}

bool isCompatibleIAJ(int fromNid, int middleNid, int toNid) {
    return isCompatibleIJ(fromNid,middleNid) and isCompatibleIJ(middleNid,toNid);
}



double compatibleIJ(int fromNid, int toNid) const {
    return  twcij[fromNid][toNid] ;
}



/*compatibility hast to be nodeid based not position based*/

const knode& getNode(int at) const {
     return original[at];
};




void recreateRowColumn( int at) {
     for (int j=0; j<twcij.size(); j++) {
         twcij[at][j]= twc_for_ij(original[at],original[j]);
         twcij[j][at]= twc_for_ij(original[j],original[at]);
     }
}


void maskHorizontal(int at) {
     for (int j=0; j<twcij.size(); j++)
         twcij[at][j]=  -std::numeric_limits<double>::max();
}

void maskVertical(int at) {
     for (int i=0; i<twcij.size(); i++)
         twcij[i][at]=  -std::numeric_limits<double>::max();
}


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
int  getBestCompatible(int fromNid, const Bucket &nodes) {
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



double ec2(int at) {
     double ec2_tot=0;
//     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( not (twcij[at][j]  == _MIN()) ) ec2_tot+=twcij[at][j];
         if ( not (twcij[j][at]  == _MIN()) ) ec2_tot+=twcij[j][at];
     };
     if ( twcij[at][at] == _MIN() ) ec2_tot-=twcij[at][at];
     return ec2_tot;
}

int countIncompatibleFrom(int at) {
     int count=0;
     for (int j=0; j<twcij.size(); j++) {
         if ( twcij[at][j]  == _MIN() ) count++;
     }
     return count;
}

int countIncompatibleTo(int at) {
     int count=0;
     for (int j=0; j<twcij.size(); j++) {
         if ( twcij[j][at]  == _MIN() ) count++;
     }
     return count;
}





/*    DUMPS   */
void dump() const  {
    dump(original);
} 


void dump(const Bucket &nodes) const  {
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

void dumpCompatible() {
    for (int i=0;i<original.size();i++) {
      for (int j=0;j<original.size();j++) {
        for (int k=0;k<original.size();k++) {
          std::cout<<"\t ( "<<original[i].getnid()<<" , "<<original[j].getnid()<<" , "<<original[k].getnid()<<") = "<<(isCompatibleIAJ(i,j,k)? "COMP": "not");
        }
        std::cout<<"\n";
      }
    }
}


const knode& node(int i) const {
    return original[i];
};

// Functions to adjust compatability depending on problem 

void setIncompatible(int fromNid,int toNid) {
    twcij[fromNid][toNid]= _MIN();
}


// constructors
TWC() {};
TWC(Bucket _original)  : original(_original) {
/*    for (int i=0; i<original.size();i++){
        nodes.push_back(original[i]);
    }
*/
    twcij_calculate();
}

/* private are indexed */
private:
double compat(int i,int j) const {
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
#ifdef DEBUG
std::cout<<" Quiero llegar a J="<<nj.getnid()<<" que abre a las:"<<nj.opens()<<" y cierra a las:"<<nj.closes()<<
"\n \tDesde:"<<ni.getnid()<<" Si llego a "<<ni.getnid()<<" a la hora que abre, entonces a "<<nj.getnid()<<" llego a las= "<<ajei(ni,nj),"\n";
#endif
    if ( ( nj.closes() -ajei(ni,nj) ) > 0 ) {
#ifdef DEBUG
std::cout<<"\n \tDesde:"<<ni.getnid()<<" Si llego a "<<ni.getnid()<<" a la hora que cierra, entonces a "<<nj.getnid()<<" llego a las= "<<ajli(ni,nj),"\n";
std::cout<<"\n \t \t min ("<<ajli(ni,nj)<<","<<nj.closes()<<")\t max("<<ajei(ni,nj)<<","<<nj.opens()<<")";
std::cout<<"\t = "<< std::min (ajli(ni,nj),nj.closes())<<"\t "<<std::max(ajei(ni,nj),nj.opens())<<"";
std::cout<<"\t = "<< std::min (ajli(ni,nj),nj.closes())-std::max(ajei(ni,nj),nj.opens())<<"";
#endif
        result = std::min ( ajli(ni,nj) , nj.closes() )
                  - std::max ( ajei(ni,nj) , nj.opens()  ) ;

    } else {
#ifdef DEBUG
std::cout<<"\t Es imposible llegar a J desde I ya que por mas temprano que salga de I no hay posibilidad de que llegue a tiempo \n";
#endif
        result= _MIN();
    }
#ifdef DEBUG
std::cout<<"\t = "<< result<<"\n";
#endif
    return result;
}



/* public functions That are id based */
void twcij_calculate(){

    twcij.resize(original.size());

    for (int i=0;i<original.size();i++) twcij[i].resize(original.size());

    for (int i=0;i<original.size();i++){
        for (int j=i; j<original.size();j++) {

#ifdef DEBUG
std::cout<<"\nworking with ("<<original[i].getnid()<<","<<original[j].getnid()<<")\n";
#endif
              twcij[i][j]= twc_for_ij(original[i],original[j]);
              twcij[j][i]= twc_for_ij(original[j],original[i]);
        }
    }
    //twc_from_depot_calculate();
}




};


#endif
