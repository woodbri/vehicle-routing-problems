
#include <limits>
#include "route.h"
//#include "solution.h"

inline void swap(int a, int b) {
    int tmp = a;
    a = b;
    b = tmp;
}

Route::Route(Problem& p) : P(p) , routePath(p.getdepot()) {
    updated = true;
    D = 0;
    TWV = 0;
    CV = 0;
    path.clear();
    orders.clear();
};


bool Route::earlyArrival(int pathstop, double D) const {
    return P.earlyArrival(path[pathstop],D);
}

bool Route::lateArrival(int pathstop, double D) const {
    return P.lateArrival(path[pathstop],D);
}

double Route::distanceToPrev(int pathstop) {
     if (pathstop == 0)  return P.distance(0, path[pathstop]);
     else  return P.distance(path[pathstop-1], path[pathstop]);
}

double Route::distanceToNext(int pathstop) {
     if (pathstop == path.size()-1)  return P.distance(0, path[pathstop]);
     else  return P.distance(path[pathstop+1], path[pathstop]);
}

int Route::nodeDemand(int pathstop) const {
    return P.nodeDemand(path[pathstop]);
}


int Route::nodeServiceTime(int pathstop) const {
    return P.nodeServiceTime(path[pathstop]);
}


bool Route::capacityViolation(double q) const {
    return (q<0 and q>P.Q);
}

void Route::update() {
    D = 0; //duration
    TWV = 0; //Time window violation
    CV = 0; //Capacity violations
    int q = 0; // current used capcity

    for (int i=0; i<path.size(); i++) {
        D += distanceToPrev(i);
        if (lateArrival(i,D)) TWV++;

        // if we arrive before the tw open time, we have to wait till then
        if (earlyArrival(i,D))  D = P.N[path[i]].opens();

        // add the demand for this node and check for violation
        q += nodeDemand(i);
        if (capacityViolation(q)) CV++;

        // add the service time for this node
        D +=nodeServiceTime(i);
    }

    // add the distance between last delivery node and depot
    if (path.size()) D += distanceToNext(path.size()-1);
    //if (D > P.DepotClose) {
    if (P.getdepot().lateArrival(D))  TWV++;
    cost = w1*D + w2*TWV + w3*CV;
    updated = false;
};





double Route::testPath(const std::vector<int>& tp) {
    tD = 0;
    tTWV = 0;
    tCV = 0;
    int q = 0; // current used capcity

    for (int i=0; i<tp.size(); i++) {
        // add the distance from the previous stop
        if (i == 0) {
            tD += P.distance(0, tp[i]);
        }
        else {
            tD += P.distance(tp[i-1], tp[i]);

            // if the current distance is > previous node close time
            if (tD > P.N[tp[i]].closes())
                tTWV++;
        }

        // if we arrive before the tw open time, we have to wait till then
        if (tD < P.N[tp[i]].opens())
            tD = P.N[tp[i]].opens();

        // add the demand for this node and check for violation
        q += P.N[tp[i]].getDemand();
        if (q < 0 || q > P.Q)
            tCV++;

        // update the capacity and pdist lists
        // capacity[i] = q; // capacity after node is loaded
        // pdist[i] = D; // distance at node max(arrival time, tw_open)

        // add the service time for this node
        tD += P.N[tp[i]].getServiceTime();
    }

    // add the distance between last delivery node and depot
    if (tp.size())
        tD += P.distance(tp[tp.size()-1], 0);
    if (tD > P.DepotClose)
        tTWV++;

    return w1*tD + w2*tTWV + w3*tCV;
};






bool Route::insertOrder(int oid, bool mustBeValid) {
/*
    // check if the order is already in the route
    std::vector<int>::iterator it;
    it = std::find(orders.begin(), orders.end(), oid);
    if (it != orders.end()) return false;

    Node& np = P.N[P.O[oid].pid];
    Node& nd = P.N[P.O[oid].did];
//    Node& np = P.getPickupNodeFromOrder(oid);
//    Node& nd = P.getDeliveryNodeFromOrder(oid);

    double bestTestCost = std::numeric_limits<double>::max();
    int bestPosition = -1;
    std::vector<int> newpath; // path with predecessor inserted
    std::vector<int> newpath2; // path with predecessot and successor inserted

    for (int i=0; i<path.size(); i++) {
        newpath = path;
        std::vector<int>::iterator it2;

        // insert the predecessor and check for violations
        it2 = newpath.begin();
        newpath.insert(it2+i, np.getnid());
        testPath(newpath);
        // a valid placement of the predessor node
        // requires that there are NO CV ot TW violations
        if (tCV > 0 || tTWV > 0) continue;

        // got a good insertion point, so now try the successor
        for (int j=i; j<path.size(); j++) {
            newpath2 = newpath;
            it2 = newpath2.begin();
            newpath2.insert(it2+j, nd.getnid());
            double tcost = testPath(newpath2);
            // if we are eliminating a route then mustBeValid is true
            // and we must be able to also place the successor node
            // without creating violations
            if (mustBeValid && (tCV > 0 || tTWV > 0)) continue;

            // if this is better than the previous best then save it
            if (tcost < bestTestCost) {
                bestTestCost = tcost;
                bestPosition = j;
            }
        }

        // TODO: why did we have the following???
        // if (bestPosition != -1) break;
    }

    // if we did not insert it above see if we can append to this route
    if (bestPosition == -1) {
        newpath2 = path;
        newpath2.push_back(np.getnid());
        newpath2.push_back(nd.getnid());
        double tcost = testPath(newpath2);
        if (mustBeValid && tCV == 0 && tTWV == 0) {
            bestTestCost = tcost;
            bestPosition = path.size();
        }
    }

    if (bestPosition != -1) {
        // apply the moves to this route
        path.clear();
        std::vector<int>::iterator it;
        for (int i=0; i<newpath2.size(); i++) {
            path.push_back(newpath2[i]);
            if (newpath2[i] == np.getnid() || newpath2[i] == nd.getnid()) {
                it = orders.begin();
                orders.insert(it+i, oid);
            }
        }
        updated = true;
//std::cout << "insertOrder(" << oid << "," << mustBeValid << "): ";
//dump();
        return true;
    }
    return false;
*/}

void Route::addOrder( const Order &o) {
    addPickup(o);
    addDelivery(o);

}


int  Route::getppos  (const int oid) const  {
    return routePath.getppos(oid);
}

int  Route::getdpos (const int oid) const {
    return routePath.getdpos(oid);
}

void Route::removeOrder(const Order &o) {
    routePath.removeOrder(o.oid);
}

void Route::removeOrder(const int oid) {
    routePath.removeOrder(oid);
}

void Route::move(int fromi,int toj) {
   //checks are missing for valid moves
   routePath.move(fromi,toj);
}


void  Route::insertPickup(const Order &o, const int at) {
    pathNode pickup(*o.pickup);
    routePath.insert(pickup,at);
}

void  Route::remove(const int at) {
    routePath.remove(at);
}

void  Route::addPickup(const Order &o) {
    pathNode pickup(*o.pickup);
    routePath.push_back(pickup);
}


void Route::addDelivery(const Order &o) {
    pathNode delivery(*o.delivery);
    routePath.push_back(delivery);
}

int Route::findForwardImprovment(const int i,double &bestcost) {
           bool improved=false;
           int bestJ=-1;
           if (isdepot(i)) return -1;
           for (int j=i+1; j<routePath.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
                  move(i,j);
                  if (getcost()<bestcost){
                             bestcost=getcost();
                             bestJ=j;
                  }
                  move(j,i);
           }
           return  bestJ;
}


void Route::findBetterForward(int &bestI, int &bestJ) {
           double bestcost=getcost();
           int j=-1;
           for (int i=1;i<routePath.size()-1;i++) { //not even bothering with the depot
              j=-1;
              j=findForwardImprovment(i,bestcost); 
              if (j>0) { //a better cost was found
                    bestI=i;
                    bestJ=j;
             }
           }
}

/*
bppos, bdpos = best pickup postition, best delivery postition
ppos, dpos = pickup postition, delivery postition
*/
int Route::findBetterDeliveryForward(const int ppos,const int dpos,double &bestcost) {
           int bestJ=-1;
           int deliveryPos=dpos;
           if ( not (ppos<dpos and sameorder(ppos,dpos) and ispickup(ppos) and isdelivery(dpos) ))  return -1; //thoerically we never get to this point if called from funtion bellow
           for (int j=ppos+1; j<routePath.size(); j++) {
                  move(deliveryPos,j); deliveryPos=j;   
                  if (getcost()<bestcost and feasable()){
                             bestcost=getcost();
                             bestJ=j;
                  }
           }
           return  bestJ;
}

/*
bppos, bdpos = best pickup postition, best delivery postition
ppos, dpos = pickup postition, delivery postition
*/
double Route::costBetterPickupBackward(int &bppos, int &bdpos) {
           double bestcost=getcost();
           int ppos=bppos;
	   int dpos=bdpos;
	   int bestI=-1;
           int bestJ=-1;
           int j;
           if ( not (ppos<dpos and sameorder(ppos,dpos) and ispickup(ppos) and isdelivery(dpos)) ) return bestcost;
           for (int i=1;i<dpos;i++) { //ensuring pickup comes before delivery
              j=-1;
              move (ppos, i); ppos=i;               
              j=findBetterDeliveryForward(ppos,dpos,bestcost);
              if (j>0) { //a better cost was found
                    bestI=i;
                    bestJ=j;
             }
           }
           if ( bestI=bestJ=-1 ) {
                 bppos=ppos;
                 bdpos=dpos;
                 return bestcost;                    //no better cost was found
           } else {
                 bppos=bestI;
                 bdpos=bestJ;
                 return bestcost;
           }
}

double Route::findBestCostBackForw(const int oid,int &bppos,int &bdpos){
          int ppos=routePath.getppos(oid);  //actual positions and costs as best
          int dpos=routePath.getdpos(oid);
          double actualcost=getcost();
          double bestcost=actualcost;
          bppos=ppos; bdpos=dpos; 
          bestcost=costBetterPickupBackward(bppos, bdpos);
          return bestcost;
}
          



bool Route::findImprovment(int i) {
           double oldcost= getcost();
           bool improved=false;
           if (isdepot(i)) return false;
           for (int j=i+1; j<routePath.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
                   swapnodes(i,j);
                   if (getcost()<oldcost)  return true;
                   else  swap(i,j);
           }
           return false;
}


void Route::hillClimbOpt() {
           double original=getcost();
           int i=0;
           while (i<routePath.size()-1) {
              if (!findImprovment(i)) i++;
              else 
                 i=0;
           }
}


void Route::dump() {
    routePath.smalldump();
    std::cout <<  " Route cost: "<< getcost();
    routePath.dump();
}

void Route::tau() {
    for (int i=0; i< routePath.size(); i++)
       std::cout<<routePath.getnid(i)<<" , ";
}

void Route::plotTau(std::vector<double> &x, std::vector<double> &y,std::vector<int> &label,std::vector<int> &color) {
    for (int i=0; i< routePath.size(); i++) {
       x.push_back(routePath.getx(i));
       y.push_back(routePath.gety(i));
       label.push_back(routePath.getnid(i));
       if (isdepot(i)) color.push_back(0xff0000);
       else if (isdelivery(i)) color.push_back(0x00ff00);
       else color.push_back(0x0000ff);
    }
}




void Route::dumppath() {
    routePath.smalldump();
}

