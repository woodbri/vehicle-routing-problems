#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "order.h"
#include "prob_pd.h"

// NON class functions for sorting

bool sortByDist(Order a, Order b)
{
    return a.getdistPickupDepot() < b.getdistPickupDepot();
}

bool sortByDistReverse(Order a, Order b)
{
    return a.getdistPickupDepot() > b.getdistPickupDepot();
}

bool sortByOid(Order a, Order b)
{
    return a.oid < b.oid;
}

bool sortByOidReverse(Order a, Order b)
{
    return a.oid > b.oid;
}


bool NodeByDistReverse(const Dpnode &a, const Dpnode b, Dpnode depot) {
   return a.distance(depot) > b.distance(depot);
}

// Class functions

unsigned int Prob_pd::getNodeCount() const { return (unsigned int) datanodes.size(); } 
unsigned int Prob_pd::getOrderCount() const { return (unsigned int) ordersList.size(); }
int Prob_pd::getOrderOid(int i) const{ ordersList[i].getoid();};
int Prob_pd::getOrderPid(int i) const{ return ordersList[i].getpid(); };
int Prob_pd::getOrderDid(int i) const{ return ordersList[i].getpid(); }; 
Dpnode& Prob_pd::getDeliveryNodeFromOrder(int oid){ return datanodes[ordersList[oid].getdid()]; }
Dpnode& Prob_pd::getPickupNodeFromOrder(int oid){ return datanodes[ordersList[oid].getpid()]; } 
double Prob_pd::nodeDemand(int i) const { return datanodes[i].getdemand(); }
bool Prob_pd::lateArrival(int nid,double D) const { return datanodes[nid].latearrival(D); }
bool Prob_pd::earlyArrival(int nid,double D) const { return datanodes[nid].earlyarrival(D); }
double Prob_pd::nodeServiceTime(int nid) const { return datanodes[nid].getservicetime(); }
Order& Prob_pd::getOrder(int oid) { return ordersList[oid]; } 

double Prob_pd::DepotToDelivery(int n1) const {
    return  (n1>=0 && n1 <= datanodes.size()) ? datanodes[datanodes[n1].getdid()].distance(depot):-1;
}

double Prob_pd::DepotToPickup(int n1) const {
    return  (n1>=0 && n1 <= datanodes.size()) ? datanodes[n1].distance(depot):-1;
}

double Prob_pd::distance(int n1,int n2) const { return datanodes[n1].distance(datanodes[n2]); }

bool Prob_pd::checkIntegrity() const {
   bool flag=true;
   int nodesCant=datanodes.size();
   int ordersCant=ordersList.size();

   if (datanodes.empty()) {
        std::cout << "Nodes is empty\n";
        flag=false; }
   else std::cout << "# of Nodes:"<<nodesCant<<"\n";

   if (ordersList.empty()) {
        std::cout << "Orders is empty\n";
        flag=false;}
   else std::cout << "# of Orders:"<<ordersCant<<"\n";

   if (ordersCant != (nodesCant-1)/2) {
        std::cout << "Expected "<<(nodesCant-1)/2<<" Orders. Found "<<ordersCant<<" Orders\n";
        flag=false;}
   else std::cout << "Found expected # of Orders\n";

   for (int i=1;i<nodesCant-1;i++) {
     flag= flag and datanodes[i].checkintegrity();
   }
}


void Prob_pd::nodesdump() {
    std::cout << "---- Nodes  --------------\n";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}
void Prob_pd::dump() {
    std::cout << "---- Problem -------------\n";
    std::cout << "K: " << K << std::endl;
    std::cout << "Q: " << Q << std::endl;
    std::cout << "---- Orders --------------\n";
    ordersList.dump();
    std::cout << "\n";
    nodesdump();
}

Prob_pd::Prob_pd(char *infile)
     {
std::cout << "---- Constructor --------------\n";
         loadProblem(infile);
     } 


/* depot must be the first node in list... rest can be anywhere*/
void Prob_pd::loadProblem(char *infile)
{
std::cout << "---- Load --------------\n";
    std::ifstream in( infile );
    std::string line;

    // read header line
    std::getline(in, line);
    std::istringstream buffer( line );
    buffer >> K;
    buffer >> Q;


    // read the nodes
    while ( getline(in, line) ) {
        Dpnode node(line);  //create node from line on file
        datanodes.push_back(node);
        if (node.isdepot()) {
            depot=node;
            depot.setoid(-1);
        }
    }
    in.close();
    sortNodeById();
    ordersList.makeOrders(datanodes,depot);
    twc.setNodes(datanodes);
    for (int i=0;i<ordersList.size();i++) twc.setIncompatible(ordersList[i]);
    Vehicle v(depot,Q);
    ordersList.setCompatibility(twc,v);
}

/* sorts */
void Prob_pd::sortNodeByTWC(){
    int j;
    Dpnode tmp;
    double twc;
std::cout<<"CALL TO SORTNODEBYTWC NOT WEORKING\n";
return;
/*
    for (int i=2; i<datanodes.size();i++) {
      tmp=datanodes[i];
      twc=twcTot[i];
      for (j = i; j > 1 and twcTot[j-1] < twc ; j-- ) { 
        datanodes[j]=datanodes[j-1]; 
        twcTot[j]=twcTot[j-1];
      }
      datanodes[j]=tmp;
      twcTot[j]=twc;
    }
*/
};

void Prob_pd::sortNodeById() {
    int j;
    Dpnode tmp;
    for (int i=2; i<datanodes.size();i++) {
      tmp=datanodes[i];
      for (j = i; j > 1 and datanodes[j-1].getnid() > tmp.getnid();j-- ) {
        datanodes[j]=datanodes[j-1];
      }
      datanodes[j]=tmp;
    }
};

void Prob_pd::sortNodeByDistReverse(){
    int j;
    Dpnode tmp;
    for (int i=2; i<datanodes.size();i++) {
      tmp=datanodes[i];
      for (j = i; j > 1 and datanodes[j-1].distance(depot) < tmp.distance(depot);j-- ) { 
        datanodes[j]=datanodes[j-1]; 
      }
      datanodes[j]=tmp;
    }
};


void Prob_pd::sortOrdersbyDistReverse(){
    sort(ordersList.begin(), ordersList.end(), sortByDistReverse);
};
void Prob_pd::sortOrdersbyId(){
    sort(ordersList.begin(), ordersList.end(), sortByOid);
};

void Prob_pd::sortOrdersbyIdReverse(){
    sort(ordersList.begin(), ordersList.end(), sortByOidReverse);
};


void Prob_pd::sortOrdersbyDist(){
std::cout<<"going to sort Orders by Dist";
    sort(ordersList.begin(), ordersList.end(), sortByDist);
std::cout<<"done sort Orders by Dist";
};


/*
void Problem::calcAvgTWLen() {
     get the average time window length
    atwl = 0;
    for (int i=0; i<N.size(); i++)
        atwl += N[i].windowLength();
    atwl /= N.size();
};
*/
