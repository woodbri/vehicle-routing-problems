
#include <iostream>
#include <vector>


#include "order.h"
#include "twpath.h"
#include "vehicle.h"



int Vehicle::findForwardImprovment(const int i,double &bestcost) {
           bool improved=false;
           int bestJ=-1;
           if (isdepot(i)) return -1;
           for (int j=i+1; j<path.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
                  move(i,j);
                  if (getcost()<bestcost){
                             bestcost=getcost();
                             bestJ=j;
                  }
                  move(j,i);
           }
           return  bestJ;
}


bool Vehicle::findImprovment(int i) {
           double oldcost= getcost();
           bool improved=false;
           if (isdepot(i)) return false;
           for (int j=i+1; j<path.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
                   swapnodes(i,j);
                   if (getcost()<oldcost)  return true;
                   else  swap(i,j);
           }
           return false;
}


void Vehicle::hillClimbOpt() {
           double original=getcost();
           int i=0;
           while (i<path.size()-1) {
              if (!findImprovment(i)) i++;
              else 
                 i=0;
           }
}



void Vehicle::findBetterForward(int &bestI, int &bestJ) {
           double bestcost=getcost();
           int j=-1;
           for (int i=1;i<path.size()-1;i++) { //not even bothering with the depot
              j=-1;
              j=findForwardImprovment(i,bestcost); 
              if (j>0) { //a better cost was found
                    bestI=i;
                    bestJ=j;
             }
           }
}



   int Vehicle::getdpos(const int oid) const {
          int at=0;
          while (at<path.size() and !(isdelivery(at) and getnid(at)==oid))
            at++; 
         return at;
    }
   int Vehicle::getppos(const int oid) const {
          int at=0;
          while (at<path.size() and !(ispickup(at) and getoid(at)==oid))
            at++; 
         return at;
    }



    void Vehicle::removeOrder(const int oid){
         removePickup(oid);
         removeDelivery(oid);
    }

    void Vehicle::removePickup(int oid){
          for (int at=0;at<path.size();at++) {
               if (ispickup(at) and getoid(at)==oid ){
                   remove(at); break; 
               }

         }
   }

    void Vehicle::removeDelivery(int oid){
           for (int at=0;at<path.size();at++) {
               if (isdelivery(at) and getoid(at)==oid ){
                   remove(at); break; //only 1 delivery per order
               }
           }
    }

    void Vehicle::remove(int at){
          if (!path.empty()) path.remove(at);
    }


    //double Vehicle::getcost(double w1,double w2,double w3) { evaluate(); return   w1*duration + w2*TWV + w3*CV; }


    void Vehicle::push_back(Dpnode pathstop) {
          path.push_back(pathstop);
    }

    void Vehicle::insert(Dpnode pathstop,int at) {
         path.insert(pathstop,at);
    }
/*
    void Vehicle::setvalues(int at){
         if (at<path.size()) {
              if (at==0) path[at].evaluate(maxcapacity);
              else path[at].evaluate(path[at-1],maxcapacity);
              setvalues(at+1);
         } else {
              setDepotValues();
         };
     }
*/

    void Vehicle::move(int fromi,int toj) {
              path.move(fromi,toj);
              //some checking migh go in route level
              //if (fromi<toj){
              //    insert(path[fromi],toj+1);
              //    remove(fromi);
              //}
               //else {
              //    insert(path[fromi],toj);
              //    remove(fromi+1);
              //}
    }

     void Vehicle::swapnodes(int i,int j){
          if(i>j)  std::cout<<"This is a restrictive swap, requierment: i<j\n";  
          else if (ispickup(i) and isdelivery(j) and sameorder(i,j)) std::cout<<"This is a restrictive swap, requierment: cant swap from the same order\n";
          else {
              path.swap(i,j);
              //Dpnode temp(path[i]);
              //path[i]=path[j];
              //path[j]=temp;
  //            setvalues(i); //update values starting from i
  //            setvalues(0);
          }
     }

     void Vehicle::swap(int i,int j){	
          path.swap(i,j);
    //      Dpnode temp(path[i]);
    //      path[i]=path[j];
    //      path[j]=temp;
    //      if (i<j) setvalues(i);
    //      else setvalues(j);
    //      setvalues(0);
     }

/*
     void Vehicle::setDepotValues() {
              int at= path.size()-1;
              //D = path[at].totDistFromDepot()+depot->distance(path[at].getnode());
              //D = path[at].totDistFromDepot+depot->distance(path[at]);
              //wv_depot=depot->lateArrival(D);
              cv_depot=path[at].getcvTot();
              TWV = path[at].gettwvTot();
              CV = path[at].getcvTot();
              TWV = (twv_depot)? TWV+1:TWV;
              CV = (cv_depot)? CV+1:CV;
      }
*/

void Vehicle::dump()  {
//     evaluate();
     for (int i=0;i<path.size();i++){
         std::cout<<"\npath stop #:"<<i<<"\n";
          //path[i].dump();
          path[i].dumpeval();
     }
     std::cout<<"\nBack to depot:"<<"\n";
     std::cout<<"twv_depot="<<twv_depot
                 <<",cv_depot="<<cv_depot
                 <<",twvTot="<<twvTot
                 <<",cvTot="<<cvTot
                 <<",current cargo="<<curcapacity
                 <<",duration="<<duration
                 <<",cost="<<cost
                 <<"\n";

     
}



void Vehicle::smalldump() {
//    evaluate();
    std::cout << "D="<<duration << ", "
              << "TWV="<<twvTot << ", "
              << "CV=" <<cvTot<< ", ";
    if(twv_depot) std::cout<<"depot: has twv ";
    if(cv_depot) std::cout<<"depot: has cv ";
    std::cout << "\nVehicle(nid,oid): [";
    for (int i=0; i<path.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << "("<<getnid(i)/*<<","<<getoid(i)<<")"*/;
    }
    std::cout << "]\n";
}

void Vehicle::evaluate() {
   evaluate(0);
};

void Vehicle::evaluate(int from) {
   
   if (from <0 or from >path.size()) from=0;
   for (int i=from; i< path.size(); i++) {
       if (i==0)path[0].evaluate(maxcapacity);
       else path[i].evaluate(path[i-1],maxcapacity);
   };
   Dpnode last=path[path.size()-1];
   if (path.size()>1) {  //if I add the depot and calculate???
     curcapacity=last.getcargo();
     duration=last.gettotDist()+depot.distance(last);
     cv_depot=curcapacity!=0;
     twv_depot=depot.latearrival(duration);
     cvTot=last.getcvTot();
     twvTot=last.gettwvTot();
     cvTot=cv_depot? cvTot:cvTot+1;
     twvTot=twv_depot? twvTot:twvTot+1;
     cost= w1*duration + w2*twvTot + w3*twvTot;
   } else {
     curcapacity=duration=cvTot=twvTot=0;
     cv_depot=twv_depot=false;
   }
}



void Vehicle::plot(std::vector<double> &x, std::vector<double> &y,std::vector<int> &label,std::vector<int> &color) {
    for (int i=0; i< path.size(); i++) {
       x.push_back(path[i].getx());
       y.push_back(path[i].gety());
       label.push_back(path[i].getnid());
       if (isdepot(i)) color.push_back(0xff0000);
       else if (isdelivery(i)) color.push_back(0x00ff00);
       else color.push_back(0x0000ff);
    }
}




void  Vehicle::insertPickup(const Order &o, const int at) {
    Dpnode pickup(*o.pickup);
    path.insert(pickup,at);
}

//void  Vehicle::remove(const int at) {
//    path.remove(at);
//}

void  Vehicle::addPickup(const Order &o) {
    Dpnode  pickup(*o.pickup);
    path.push_back(pickup);
    evalLast();
}


void Vehicle::addDelivery(const Order &o) {
    Dpnode delivery(*o.delivery);
    path.push_back(delivery);
    evalLast();
}

void Vehicle::addOrder( const Order &o) {
    addPickup(o);
    addDelivery(o);
}

void Vehicle::tau() {
    for (int i=0; i< path.size(); i++)
       std::cout<<getnid(i)<<" , ";
}

void Vehicle::evalLast() {
    evaluate(path.size()-1);
}



/*
#include <limits>
#include "route.h"
//#include "solution.h"

inline void swap(int a, int b) {
    int tmp = a;
    a = b;
    b = tmp;
}

Route::Route(Problem& p) : P(p) , routeVehicle(p.getdepot()) {
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





double Route::testVehicle(const std::vector<int>& tp) {
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
        testVehicle(newpath);
        // a valid placement of the predessor node
        // requires that there are NO CV ot TW violations
        if (tCV > 0 || tTWV > 0) continue;

        // got a good insertion point, so now try the successor
        for (int j=i; j<path.size(); j++) {
            newpath2 = newpath;
            it2 = newpath2.begin();
            newpath2.insert(it2+j, nd.getnid());
            double tcost = testVehicle(newpath2);
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
        double tcost = testVehicle(newpath2);
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
}



int  Route::getppos  (const int oid) const  {
    return routeVehicle.getppos(oid);
}

int  Route::getdpos (const int oid) const {
    return routeVehicle.getdpos(oid);
}

void Route::removeOrder(const Order &o) {
    routeVehicle.removeOrder(o.oid);
}

void Route::removeOrder(const int oid) {
    routeVehicle.removeOrder(oid);
}

void Route::move(int fromi,int toj) {
   //checks are missing for valid moves
   routeVehicle.move(fromi,toj);
}


void Route::findBetterForward(int &bestI, int &bestJ) {
           double bestcost=getcost();
           int j=-1;
           for (int i=1;i<routeVehicle.size()-1;i++) { //not even bothering with the depot
              j=-1;
              j=findForwardImprovment(i,bestcost); 
              if (j>0) { //a better cost was found
                    bestI=i;
                    bestJ=j;
             }
           }
}


//bppos, bdpos = best pickup postition, best delivery postition
//ppos, dpos = pickup postition, delivery postition

int Route::findBetterDeliveryForward(const int ppos,const int dpos,double &bestcost) {
           int bestJ=-1;
           int deliveryPos=dpos;
           if ( not (ppos<dpos and sameorder(ppos,dpos) and ispickup(ppos) and isdelivery(dpos) ))  return -1; //thoerically we never get to this point if called from funtion bellow
           for (int j=ppos+1; j<routeVehicle.size(); j++) {
                  move(deliveryPos,j); deliveryPos=j;   
                  if (getcost()<bestcost and feasable()){
                             bestcost=getcost();
                             bestJ=j;
                  }
           }
           return  bestJ;
}


//bppos, bdpos = best pickup postition, best delivery postition
//ppos, dpos = pickup postition, delivery postition

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
          int ppos=routeVehicle.getppos(oid);  //actual positions and costs as best
          int dpos=routeVehicle.getdpos(oid);
          double actualcost=getcost();
          double bestcost=actualcost;
          bppos=ppos; bdpos=dpos; 
          bestcost=costBetterPickupBackward(bppos, bdpos);
          return bestcost;
}
          

void Route::dump() {
    routeVehicle.smalldump();
    std::cout <<  " Route cost: "<< getcost();
    routeVehicle.dump();
}


void Route::dumppath() {
    routeVehicle.smalldump();
}
*/
