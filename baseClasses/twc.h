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
typedef Twpath<knode> Bucket;


    Twpath<knode> nodes;
    Bucket original;


    std::map<int,int> IdPos;
    std::deque<std::deque<double> > twcij;


    std::deque<double>  twc0;

    inline double _MIN() const { return -std::numeric_limits<double>::max();};
/* CLASS SUMMARY


    double ajli(const knode &ni, const knode &nj);
    double ajei(const knode &ni, const knode &nj);
    double twc_for_ij(const knode &ni, const knode &nj);
    double compat(int i,int j) const ;


  public:
    TWC() {};
    TWC(Bucket _original);
    int setSubset(Bucket _subset);
    int setNodes(Bucket _original);
  
    knode getNode(int nodeId);

        
    void setIncompatible(int fromId, int toId);
    void twcij_calculate();
    void twc_from_depot_calculate();
// compatibility based on id not on position 
    bool isCompatibleIJ(int iId, int jId) const ;
    bool isCompatibleIAJ(int iId, int aId, int jId) ;
    double compatibleIJ(int iId,int jId) const ;
    void dumpTWC();
    void maskHorizontal(int at) ;
    void maskVertical(int at) ;

//The following needs a lot of work
    int  getBestCompatible(int from) ;
    int  getSeed(int fromNid, const Bucket &nodes);
    int  getBestCompatible(int fromNid,const Bucket &nodes);
    int  getBestCompatible() ;
    void recreateRowColumn( int nodeId );
    double ec2(int nodeId);
    int countIncompatibleFrom(int nodeId);
    int countIncompatibleTo(int nodeId);
    int getIdOfWorseCount(int subsetId) ;



    const knode& node(int i) const;
    void dump() const;

*/



/*
double TWC::ajli(const knode &ni, const knode &nj) {
    return ni.closes()+ni.getservicetime()+nj.distance(ni);
}

double TWC::ajei(const knode &ni, const knode &nj) {
    return ni.opens()+ni.getservicetime()+nj.distance(ni);
}


double TWC::twc_for_ij(const knode &ni, const knode &nj) {
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
*/

/*  CONSTRUCTORS */
public:
TWC() {};
TWC(Bucket _original) {
    original=_original;
    for (int i=0; i<original.size();i++){
        nodes.push_back(original[i]);
        IdPos[original.getnid(i)]=i;
    }
    twcij_calculate();
}

/*
int setSubset(Bucket _subset) {
    subset.push_back(_subset);
    return subset.size()-1;
}
*/

int setNodes(Bucket _original) {
    original=_original;
    nodes.resize(0);
    for (int i=0; i<original.size();i++){
        nodes.push_back(original[i]);
        IdPos[nodes[i].getnid()]=i;
    }
    twcij_calculate();
}




/* public functions That are id based */
void twcij_calculate(){

    twcij.resize(nodes.size());

    for (int i=0;i<nodes.size();i++) twcij[i].resize(nodes.size());

    for (int i=0;i<nodes.size();i++){
        for (int j=i; j<nodes.size();j++) {

#ifdef DEBUG
std::cout<<"\nworking with ("<<nodes[i].getnid()<<","<<nodes[j].getnid()<<")\n";
#endif
              twcij[i][j]= twc_for_ij(nodes[i],nodes[j]);
              twcij[j][i]= twc_for_ij(nodes[j],nodes[i]);
        }
    }
    twc_from_depot_calculate();
}

/* public functions are id based */
/* general */
void setIncompatible(int fromNid,int toNid) {
    int atFrom = IdPos[fromNid];
    int atTo = IdPos[toNid];
    twcij[atFrom][atTo]= _MIN();
}


bool isCompatibleIJ(int fromNid, int toNid) const {
    int atFrom = IdPos.find(fromNid)->second;
    int atTo = IdPos.find(toNid)->second;
//std::cout<<"Comparing "<<fromNid<<" with " <<toNid<<" gives "<< twcij[atFrom][atTo]<<"\n";
    return not (twcij[atFrom][atTo]  == _MIN());
}

bool isCompatibleIAJ(int fromNid, int middleNid, int toNid) {
    int atFrom = IdPos[fromNid];
    int atMiddle = IdPos[middleNid];
    int atTo = IdPos[toNid];
    return isCompatibleIJ(atFrom,atMiddle) and isCompatibleIJ(atMiddle,atTo);
}



double compatibleIJ(int fromNid, int toNid) const {
    int atFrom = IdPos.find(fromNid)->second;
    int atTo = IdPos.find(toNid)->second;
    return twcij[atFrom][atTo];
}



/*compatibility hast to be nodeid based not position based*/

knode getNode(int nid){
     int at = IdPos[nid];
     return nodes[at];
};




void recreateRowColumn( int nid) {
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         twcij[at][j]= twc_for_ij(nodes[at],nodes[j]);
         twcij[j][at]= twc_for_ij(nodes[j],nodes[at]);
     }
}


void maskHorizontal(int nid) {
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++)
         twcij[at][j]=  -std::numeric_limits<double>::max();
}

void maskVertical(int nid) {
     int at = IdPos[nid];
     for (int i=0; i<twcij.size(); i++)
         twcij[i][at]=  -std::numeric_limits<double>::max();
}

/* asummes the node is already being used therefore it masks vertical (aka.. no longer a node can reach
the fromNid node */
int  getBestCompatible(int fromNid) {
     int best=0;
     int from = IdPos[fromNid];
     maskVertical(fromNid);           //mask based on nodes id instead of position
     original.removeNode(fromNid);
     for (int j=0; j<twcij.size(); j++)
         if (twcij[from][j]>twcij[from][best])
            best=j;
     if (compat(from,best)!=_MIN()) return nodes[best].getnid();
     else return -1;
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
//         if (ec2(Id) <  bestEc2){
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

/* el que tenga mas compatibilidades
int  getSeed(int fromNid,const Bucket &nodes) {
     int best=0,bestId;
     int from = IdPos[fromNid];
     int toPos,toId;
     maskVertical(fromNid);           //mask based on nodes id instead of position
     bestId=nodes[0].getnid();
     best=IdPos[bestId];
std::cout<<"gettingseed from"<<from;
dump();
std::cout<<"\n Best twcij="<<twcij[from][best];
     for (int j=0; j<nodes.size(); j++) {
         toId = nodes[j].getnid();
         toPos= IdPos[toId];
std::cout<<"\n working with node "<<toId<<" twcij="<<twcij[from][toPos];
std::cout<<"\t < twcij="<<twcij[from][best];
             if ( compat(from,toPos) and twcij[from][toPos]<twcij[from][best]) {
                best=toPos; bestId=toId;
std::cout<<"\t  twcij="<<twcij[from][best];

             }
std::cout<<"\t best one is "<<bestId;
     }
     return bestId;
//     if (compat(from,best)!=_MIN()) return bestId;//nodes[best].getnid();
//     else return -1;
}

*/
int  getBestCompatible(int fromNid,const Bucket &nodes) {
     int best,bestId;
     int from = IdPos[fromNid];
     int toPos,toId;
     //  maskVertical(fromNid); 
//std::cout<<"\n gettingbest from "<<fromNid<<"\t";
//nodes.tau();
     if (nodes.empty()) return -1;
     bestId=nodes[0].getnid();
//std::cout<<"\t best one is "<<bestId;
     best=IdPos[bestId];
//std::cout<<"\t has "<<twcij[from][best];

     for (int j=0; j<nodes.size(); j++) {
         toId = nodes[j].getnid();
         toPos= IdPos[toId];
//std::cout<<"\n working with node "<<toId;
//std::cout<<"\t has "<<twcij[from][toPos];
             if ( twcij[from][toPos]>twcij[from][best]) {
                best=toPos; bestId=toId;
             }
//std::cout<<"\t best one is "<<bestId;
//std::cout<<"\t has "<<twcij[from][best];
     }
     if (compat(from,best)!=_MIN()) return bestId;//nodes[best].getnid();
     else return -1;
}



double ec2(int nid) {
     double ec2_tot=0;
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( not (twcij[at][j]  == _MIN()) ) ec2_tot+=twcij[at][j];
         if ( not (twcij[j][at]  == _MIN()) ) ec2_tot+=twcij[j][at];
     };
     if ( twcij[at][at] == _MIN() ) ec2_tot-=twcij[at][at];
     return ec2_tot;
}

int countIncompatibleFrom(int nid) {
     int count=0;
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( twcij[at][j]  == _MIN() ) count++;
     }
     return count;
}
int countIncompatibleTo(int nid) {
     int count=0;
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( twcij[j][at]  == _MIN() ) count++;
     }
     return count;
}


/*probably unnesesary*/
int  getBestCompatible() {
     int best=0;
     for (int j=0; j<twc0.size(); j++)
         if (twc0[j]>twc0[best])
            best=j;
     return best;
}

/* specific */




//twc0 has the horizttntal line of twcij[0]
void twc_from_depot_calculate(){
    twc0=twcij[0];
}


void dump() const  {
    std::cout<<"\n\t";
//    for (int i=0;i<twc0.size();i++)
//        std::cout<<twc0[i]<<"\t";
//    std::cout<<"\n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"pos "<<i<<"\t";
    std::cout<<"\n\t";
    for (int i=0;i<nodes.size();i++)
        std::cout<<"id "<<nodes[i].getnid()<<"\t";
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





/*    DUMPS   */
void dumpCompatible() {
    for (int i=0;i<nodes.size();i++) {
      for (int j=0;j<nodes.size();j++) {
        for (int k=0;k<nodes.size();k++) {
          std::cout<<"\t ( "<<nodes[i].getnid()<<" , "<<nodes[j].getnid()<<" , "<<nodes[k].getnid()<<") = "<<(isCompatibleIAJ(i,j,k)? "COMP": "not");
        }
        std::cout<<"\n";
      }
    }
}


const knode& node(int i) const {
    return original[i];
};

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


};


#endif
