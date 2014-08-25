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
bool Prob_pd::isAsignedOrder(int oid) const { return ordersList[oid].isAsigned(); }
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

void Prob_pd::ordersdump() {
    std::cout << "---- Orders --------------\n";
    for (int i=0; i<ordersList.size(); i++)
        ordersList[i].dump();
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
    ordersdump();
    std::cout << "\n";
    nodesdump();
}


void Prob_pd::loadProblem(char *infile)
{
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
    makeOrders();
}

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

void Prob_pd::makeOrders ()
{

    if (getNodeCount() == 0 || ((getNodeCount()-1)%2 != 0)) {
        std::string errmsg = "Problem::makeOrders - Nodes have not be correctly loaded.";
        throw std::runtime_error(errmsg);
    }

    int oid = 0;
    Order order;
    // for each pickup, get its delivery and create an order
    for (int i=0; i<getNodeCount(); i++) {
        if (datanodes[i].isdepot() or datanodes[i].isdelivery()) continue;
           int j;
           for (j=0; j<getNodeCount() and datanodes[j].getnid()!=datanodes[i].getdid(); j++) {}
           order.fillOrder(datanodes[i],datanodes[j],oid++,depot);
           ordersList.push_back(order);
    }
}

/*
void Problem::calcAvgTWLen() {
     get the average time window length
    atwl = 0;
    for (int i=0; i<N.size(); i++)
        atwl += N[i].windowLength();
    atwl /= N.size();
};
*/
