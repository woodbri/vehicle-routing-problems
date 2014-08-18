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
    //return N[N[n1].did].distance(depot);
}

double Prob_pd::DepotToPickup(int n1) const {
    return  (n1>=0 && n1 <= N.size()) ? datanodes[n1].distance(depot):-1;
    //return N[n1].distance(depot);
}

double Prob_pd::distance(int n1,int n2) const {
    return N[n1].distance(N[n2]);
/*    double dx = N[n2].x - N[n1].x;
    double dy = N[n2].y - N[n1].y;
    return sqrt( dx*dx + dy*dy );*/
}

bool Prob_pd::checkIntegrity() const {
   bool flag=true;
   int nodesCant=datanodes.size();
   int ordersCant=ordersList.size();

   if (N.empty()) {
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

  for (std::vector<Dpnode>::const_iterator it= datanodes.begin(); it!=datanodes.end(); ++it) {
     bool flag1=true;
     Dpnode node=*it;
     flag= flag and node.checkintegrity();
   }
  for (std::deque<Order>::const_iterator it= ordersList.begin(); it!=ordersList.end(); ++it) {
     Order order=*it;
     flag= flag and order.checkIntegrity(nodesCant);
   }
   return flag;
}

void Prob_pd::ordersdump() {
    std::cout << "---- Orders --------------\n";
    for (int i=0; i<ordersList.size(); i++)
        ordersList[i].dump();
}

void Prob_pd::nodesdump() {
    std::cout << "---- Nodes  --------------\n";
    for (int i=0; i<N.size(); i++)
        datanodes[i].dump();
}
void Prob_pd::dump() {
    std::cout << "---- Problem -------------\n";
    std::cout << "K: " << K << std::endl;
    std::cout << "Q: " << Q << std::endl;
    std::cout << "w1: " << w1 << std::endl;
    std::cout << "w2: " << w2 << std::endl;
    std::cout << "w3: " << w3 << std::endl;
    /*std::cout << "extents: " << extents[0] << ", "
                             << extents[1] << ", "
                             << extents[2] << ", "
                             << extents[3] << std::endl;*/
    ordersdump();
    nodesdump();
/*
    std::cout << "---- Orders --------------\n";
    for (int i=0; i<O.size(); i++)
        O[i].dump();
    std::cout << "---- Nodes  --------------\n";
    for (int i=0; i<N.size(); i++)
        N[i].dump();
*/
    std::cout << std::endl;
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

    // initialize the extents
    //extents[0] = std::numeric_limits<double>::max();
    //extents[1] = std::numeric_limits<double>::max();
    //extents[2] = std::numeric_limits<double>::min();
    //extents[3] = std::numeric_limits<double>::min();


    // read the nodes
    while ( getline(in, line) ) {
        Dpnode node(line);  //create node from line on file

        // compute the extents as we load the data for plotting 
      //  if (node.getx() < extents[0]) extents[0] = node.getx();
      //  if (node.gety() < extents[1]) extents[1] = node.gety();
      //  if (node.getx() > extents[2]) extents[2] = node.getx();
      //  if (node.gety() > extents[3]) extents[3] = node.gety();

        datanodes.push_back(node);
        //if (node.nid == 0)
        if (node.isdepot()) {
            DepotClose = node.closes();
            depot=node;
        }
    }

    in.close();

    // add a small buffer around the extents
    //extents[0] -= (extents[2] - extents[0]) * 0.02;
    //extents[2] += (extents[2] - extents[0]) * 0.02;
    //extents[1] -= (extents[3] - extents[1]) * 0.02;
    //extents[3] += (extents[3] - extents[1]) * 0.02;

    // make orders from the nodes
    makeOrders();

    // sort the orders
    //calcAvgTWLen();
}

void Prob_pd::sortOrdersbyDistReverse(){
    sort(ordersList.begin(), ordersList.end(), sortByDistReverse);
};

void Prob_pd::sortOrdersbyDist(){
    sort(ordersList.begin(), ordersList.end(), sortByDist);
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
        if (datanodes[i].isdepot()) continue;  //no order for depot}
        if (datanodes[i].ispickup()) {
              order.fillOrder(datanodes[i],datanodes[datanodes[i].getdid()],oid++,depot);
              ordersList.push_back(order);

        }
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
