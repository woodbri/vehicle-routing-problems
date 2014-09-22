#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "order.h"
#include "prob_pd.h"

// Class functions

/*
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
*/
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

void Prob_pd::plot(Plot<Dpnode> &graph) {
    for (int i=0; i<datanodes.size(); i++){
        if (datanodes[i].ispickup())  {
             graph.drawPoint(datanodes[i], 0x0000ff, 9, true);
        } else if (datanodes[i].isdelivery()) {
             graph.drawPoint(datanodes[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(datanodes[i], 0xff0000, 7, true);
        }
    }
};

Prob_pd::Prob_pd(char *infile)
     {
std::cout << "---- Constructor --------------\n";
         loadProblem(infile);
     } 


/* depot must be the first node in list... rest can be anywhere*/
void Prob_pd::loadProblem(char *infile)
{
    datafile=std::string(infile);
std::cout << "---- Load --------------";
std::cout << datafile<< " ---- Load --------------\n";

    std::ifstream in( infile );
    std::string line;

    // read header line
    std::getline(in, line);
    std::istringstream buffer( line );
    buffer >> K;
    buffer >> Q;


    // read the nodes
    int nid=0;
    while ( getline(in, line) ) {
        Dpnode node(line);  //create node from line on file
        node.setnid(nid);
        datanodes.push_back(node);
        if (node.isdepot()) {
            depot=node;
            depot.setoid(-1);
        }
        nid++;
    }
    in.close();
    ordersList.makeOrders(datanodes,depot);
    twc.setNodes(datanodes);
    for (int i=0;i<ordersList.size();i++) {
          twc.setIncompatible(ordersList[i].getdid(), ordersList[i].getpid());
          twc.setUnreachable(ordersList[i].getdid(), ordersList[i].getpid());
    }
    Vehicle v(depot,Q);
    ordersList.setCompatibility(twc,v);
}


