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


/*compatibility

double Prob_pd::ajli(const Dpnode &ni, const Dpnode &nj) {
    return ni.closes()+ni.getservicetime()+nj.distance(ni);
}

double Prob_pd::ajei(const Dpnode &ni, const Dpnode &nj) {
    return ni.opens()+ni.getservicetime()+nj.distance(ni);
}


double Prob_pd::twc_for_ij(const Dpnode &ni, const Dpnode &nj) {
    double result;
//std::cout<<" Quiero llegar a J="<<nj.getnid()<<" que abre a las:"<<nj.opens()<<" y cierra a las:"<<nj.closes()<<
//"\n \tDesde:"<<ni.getnid()<<" Si llego a "<<ni.getnid()<<" a la hora que abre, entonces a "<<nj.getnid()<<" llego a las= "<<ajei(ni,nj),"\n";
    if ( ( nj.closes() -ajei(ni,nj) ) > 0 ) {
//std::cout<<"\n \tDesde:"<<ni.getnid()<<" Si llego a "<<ni.getnid()<<" a la hora que cierra, entonces a "<<nj.getnid()<<" llego a las= "<<ajli(ni,nj),"\n";
//std::cout<<"\n \t \t min ("<<ajli(ni,nj)<<","<<nj.closes()<<")\t max("<<ajei(ni,nj)<<","<<nj.opens()<<")";
//std::cout<<"\t = "<< std::min (ajli(ni,nj),nj.closes())<<"\t "<<std::max(ajei(ni,nj),nj.opens())<<"";
//std::cout<<"\t = "<< std::min (ajli(ni,nj),nj.closes())-std::max(ajei(ni,nj),nj.opens())<<"";
        result = std::min ( ajli(ni,nj) , nj.closes() )
                  - std::max ( ajei(ni,nj) , nj.opens()  ) ;
        
    } else {
//std::cout<<"\t Es imposible llegar a J desde I ya que por mas temprano que salga de I no hay posibilidad de que llegue a tiempo \n";
        result= -std::numeric_limits<double>::max();
    }
//std::cout<<"\t = "<< result<<"\n";
    return result;
}



void Prob_pd::twcij_calculate(){
    twcij.resize(datanodes.size());
    for (int i=0;i<datanodes.size();i++)
        twcij[i].resize(datanodes.size());
    for (int i=0;i<datanodes.size();i++){
        for (int j=i; j<datanodes.size();j++) {
//std::cout<<"\nworking with ("<<datanodes[i].getnid()<<","<<datanodes[j].getnid()<<")\n";
           //if (i==j) {
           //   twcij[i][j]= -std::numeric_limits<double>::max();  //mismo nodo
           //} else  {
              twcij[i][j]= twc_for_ij(datanodes[i],datanodes[j]);
              twcij[j][i]= twc_for_ij(datanodes[j],datanodes[i]);
              
              if  (i!=j and datanodes[i].getoid()==datanodes[j].getoid()){  // misma orden
                 if (datanodes[i].isdelivery() and datanodes[j].ispickup())
                    twcij[i][j]= -std::numeric_limits<double>::max();
                 else
                    twcij[j][i]= -std::numeric_limits<double>::max();
              }
              
           //}
        }
    }
    twcTot_calculate();
}

bool Prob_pd::compatibleIJ(int i, int j){
    return not (twcij[i][j]  == -std::numeric_limits<double>::max());
}

bool Prob_pd::compatibleIAJ(int i, int a, int j){
    return compatibleIJ(i,a) and compatibleIJ(a,j);
}


void Prob_pd::dumpCompatible() {
    for (int i=0;i<datanodes.size();i++) {
      for (int j=0;j<datanodes.size();j++) {
        for (int k=0;k<datanodes.size();k++) {
          std::cout<<"\t ( "<<datanodes[i].getnid()<<" , "<<datanodes[j].getnid()<<" , "<<datanodes[k].getnid()<<") = "<<(compatibleIAJ(i,j,k)? "COMP": "not");
        }
        std::cout<<"\n";
      }
    }
}

void Prob_pd::recreateRowColumn( int nid) {
     int at = originalnodes.getpos(nid);
     for (int j=0; j<twcij.size(); j++) {
         twcij[at][j]= twc_for_ij(datanodes[at],datanodes[j]);
         twcij[j][at]= twc_for_ij(datanodes[j],datanodes[at]);
     }
}


void Prob_pd::maskHorizontal(int at) {
     for (int j=0; j<twcij.size(); j++) 
         twcij[at][j]=  -std::numeric_limits<double>::max();
}

void Prob_pd::maskVertical(int at) {
     for (int i=0; i<twcij.size(); i++) 
         twcij[i][at]=  -std::numeric_limits<double>::max();
}

int  Prob_pd::getBestCompatible(int from) {
     int best=0;
     for (int j=0; j<twcij.size(); j++) 
         if (twcij[from][j]>twcij[from][best])
            best=j;
     return best;
}

int  Prob_pd::getBestCompatible() {
     int best=0;
     for (int j=0; j<twcTot.size(); j++) 
         if (twcTot[j]>twcTot[best])
            best=j;
     return best;
}


int  Prob_pd::getBestPickupCompatible(int from) {
     int best=0;
     for (int j=0; j<twcij.size(); j++) 
         if (twcij[from][j]>twcij[from][best] and datanodes[j].ispickup())
            best=j;
     return best;
}

int  Prob_pd::getBestPickupCompatible() {
     int best=0;
     for (int j=0; j<twcTot.size(); j++) 
         if (twcTot[j]>twcTot[best] and datanodes[j].ispickup())
            best=j;
     return best;
}






//twcTot has the horizontal line of twcij[0]
void Prob_pd::twcTot_calculate(){
    twcTot=twcij[0];
}


void Prob_pd::twcijDump() const  {
    std::cout<<"\n\t";
    for (int i=0;i<twcTot.size();i++)
        std::cout<<twcTot[i]<<"\t";
    std::cout<<"\n\t";
    for (int i=0;i<datanodes.size();i++)
        std::cout<<"pos "<<i<<"\t";
    std::cout<<"\n\t";
    for (int i=0;i<datanodes.size();i++)
        std::cout<<"id "<<datanodes[i].getnid()<<"\t";
    std::cout<<"\n";
    for (int i=0;i<datanodes.size();i++){
        std::cout<<datanodes[i].getnid()<<"\t";
        for (int j=0; j<datanodes.size();j++) {
           if (twcij[i][j] !=  -std::numeric_limits<double>::max()) std::cout<<twcij[i][j]<<"\t";
           else std::cout<<"--\t";
        }
        std::cout<<"\n";
    }
}


double Prob_pd::compat(int i,int j) const {
    return twcij[i][j];
};
*/
/* depot must be the first node in list... rest can be anywhere*/
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
    for (int i=0;i<datanodes.size();i++)
        originalnodes.push_back(datanodes[i]);
    twc.setNodes(originalnodes);
twc.dump();
//    twcij_calculate();
//    sortNodeByTWC();
//twcijDump();
//dumpCompatible();
//    twcij_calculate();
//twcijDump();

}

/* sorts */
void Prob_pd::sortNodeByTWC(){
    int j;
    Dpnode tmp;
    double twc;
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

void Prob_pd::makeOrders ()
{

    if (getNodeCount() == 0 || ((getNodeCount()-1)%2 != 0)) {
        std::string errmsg = "Problem::makeOrders - Nodes have not be correctly loaded.";
        throw std::runtime_error(errmsg);
    }
    ordersList.resize(0);
    int oid = 0;
    Order order;
    // for each pickup, get its delivery and create an order
    for (int i=0; i<getNodeCount(); i++) {
        if (datanodes[i].isdepot() or datanodes[i].isdelivery()) continue;
           int j;
           for (j=0; j<getNodeCount() and datanodes[j].getnid()!=datanodes[i].getdid(); j++) {}
           order.fillOrder(datanodes[i],datanodes[j],oid++,depot);
order.debugdump();
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
