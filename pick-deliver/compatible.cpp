#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "order.h"
#include "compatible.h"


inline double _MIN() { return -std::numeric_limits<double>::max();};


/*protected functions are position based*/

double Compatible::ajli(const Dpnode &ni, const Dpnode &nj) {
    return ni.closes()+ni.getservicetime()+nj.distance(ni);
}

double Compatible::ajei(const Dpnode &ni, const Dpnode &nj) {
    return ni.opens()+ni.getservicetime()+nj.distance(ni);
}


double Compatible::twc_for_ij(const Dpnode &ni, const Dpnode &nj) {
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


/*  CONSTRUCTORS */
Compatible::Compatible(Vehicle _original) {
    original=_original;
    for (int i=0; i<original.size();i++){
        nodes.push_back(original[i]);
        IdPos[original.getnid(i)]=i;
    }
    twcij_calculate();
}

int Compatible::setSubset(Vehicle _subset) {
    subset.push_back(_subset);
    return subset.size()-1;    
}

int Compatible::setNodes(Vehicle _original) {
    original=_original;
    nodes.resize(0);
    for (int i=0; i<original.size();i++){
        nodes.push_back(original[i]);
        IdPos[original.getnid(i)]=i;
    }
    twcij_calculate();
}




/* public functions That are id based */
void Compatible::twcij_calculate(){

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
void Compatible::setIncompatible(int fromNid,int toNid) {
    int atFrom = IdPos[fromNid];
    int atTo = IdPos[toNid];
    twcij[atFrom][atTo]= _MIN();
}



bool Compatible::compatibleIJ(int fromNid, int toNid){
    int atFrom = IdPos[fromNid];
    int atTo = IdPos[toNid];
    return not (twcij[atFrom][atTo]  == _MIN());
}

bool Compatible::compatibleIAJ(int fromNid, int middleNid, int toNid){
    int atFrom = IdPos[fromNid];
    int atMiddle = IdPos[middleNid];
    int atTo = IdPos[toNid];
    return compatibleIJ(atFrom,atMiddle) and compatibleIJ(atMiddle,atTo);
}






/*compatibility hast to be nodeid based not position based*/
void Compatible::recreateRowColumn( int nid) {
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         twcij[at][j]= twc_for_ij(nodes[at],nodes[j]);
         twcij[j][at]= twc_for_ij(nodes[j],nodes[at]);
     }
}


void Compatible::maskHorizontal(int nid) {
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) 
         twcij[at][j]=  -std::numeric_limits<double>::max();
}

void Compatible::maskVertical(int nid) {
     int at = IdPos[nid];
     for (int i=0; i<twcij.size(); i++) 
         twcij[i][at]=  -std::numeric_limits<double>::max();
}

int  Compatible::getBestCompatible(int fromNid) {
     int best=0;
     int from = IdPos[fromNid];
     for (int j=0; j<twcij.size(); j++) 
         if (twcij[from][j]>twcij[from][best])
            best=j;
     return best;
}


double Compatible::ec2(int nid) {
     double ec2_tot=0;
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( not (twcij[at][j]  == _MIN()) ) ec2_tot+=twcij[at][j];
         if ( not (twcij[j][at]  == _MIN()) ) ec2_tot+=twcij[j][at];
     };
     if ( twcij[at][at] == _MIN() ) ec2_tot-=twcij[at][at];
     return ec2_tot;
}

int Compatible::countIncompatibleFrom(int nid) {
     int count=0;
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( twcij[at][j]  == _MIN() ) count++;
     }
     return count;
}

int Compatible::countIncompatibleTo(int nid) {
     int count=0;
     int at = IdPos[nid];
     for (int j=0; j<twcij.size(); j++) {
         if ( twcij[j][at]  == _MIN() ) count++;
     }
     return count;
}

int Compatible::getIdOfWorseCount(int subsetId) {
    double worse = _MIN();
    int pos = -1;
    int nid;
    for (int i=0;i<subset[subsetId].size();i++) {
        nid =subset[subsetId][i].getnid();
        if (countIncompatibleFrom(nid)>worse) {
           worse = countIncompatibleFrom(i);
           pos  = i;
        }
    }
    return subset[subsetId][pos].getnid();
}
    

/*probably unnesesary*/
int  Compatible::getBestCompatible() {
     int best=0;
     for (int j=0; j<twc0.size(); j++) 
         if (twc0[j]>twc0[best])
            best=j;
     return best;
}

/* specific */
void Compatible::setIncompatible(const Order& order) {
    int pid = order.getpid();
    int did = order.getdid();
    setIncompatible(did,pid);
}


int  Compatible::getBestPickupCompatible(int from) {
     int best=0;
     for (int j=0; j<twcij.size(); j++) 
         if (twcij[from][j]>twcij[from][best] and nodes[j].ispickup())
            best=j;
     return best;
}

int  Compatible::getBestPickupCompatible() {
     int best=0;
     for (int j=0; j<twc0.size(); j++) 
         if (twc0[j]>twc0[best] and nodes[j].ispickup())
            best=j;
     return best;
}






//twc0 has the horizttntal line of twcij[0]
void Compatible::twc_from_depot_calculate(){
    twc0=twcij[0];
}


void Compatible::dump() const  {
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


double Compatible::compat(int i,int j) const {
    return twcij[i][j];
};




/*    DUMPS   */
void Compatible::dumpCompatible() {
    for (int i=0;i<nodes.size();i++) {
      for (int j=0;j<nodes.size();j++) {
        for (int k=0;k<nodes.size();k++) {
          std::cout<<"\t ( "<<nodes[i].getnid()<<" , "<<nodes[j].getnid()<<" , "<<nodes[k].getnid()<<") = "<<(compatibleIAJ(i,j,k)? "COMP": "not");
        }
        std::cout<<"\n";
      }
    }
}
