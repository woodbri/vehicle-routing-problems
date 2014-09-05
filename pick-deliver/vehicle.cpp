#include <iostream>
#include <deque>


#include "order.h"
#include "twpath.h"
#include "vehicle.h"
#include "plot1.h"

#include <sstream>




int Vehicle::findBetterDeliveryForward(const int ppos,const int dpos,double &bestcost) {
           int bestJ=-1;
           int deliveryPos=dpos;
           if ( not (ppos<dpos and sameorder(ppos,dpos) and ispickup(ppos) and isdelivery(dpos) ))  return -1; //thoerically we never get to this point if called from funtion bellow
           for (int j=ppos+1; j<path.size(); j++) {
                  move(deliveryPos,j); deliveryPos=j;   
                  if (getcost()<bestcost and feasable()){
                             bestcost=getcost();
                             bestJ=j;
                  }
           }
           return  bestJ;
}


double Vehicle::costBetterPickupBackward(int &bppos, int &bdpos) {
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
          


double Vehicle::findBestCostBackForw(const int oid,int &bppos,int &bdpos){
    int ppos=getppos(oid);  //actual positions and costs as best
    int dpos=getdpos(oid);
    double actualcost=getcost();
    double bestcost=actualcost;
    bppos=ppos; bdpos=dpos; 
    bestcost=costBetterPickupBackward(bppos, bdpos);
    return bestcost;
}




/***************************** DUMP PRINTS PLOTS   ********/

   void Vehicle::dump() const {
     for (int i=0;i<path.size();i++){
         std::cout<<"\npath stop #:"<<i<<"\n";
          path[i].dumpeval();
     }
     std::cout<<"\nBack to depot:"<<"\n";
     backToDepot.dumpeval();
     std::cout <<"TOTAL COST="<<cost <<"\n";
   }

   Twpath<Dpnode>& Vehicle::Path() {
      return path;
   }

   Twpath<Dpnode> Vehicle::getpath() const  {
      Twpath<Dpnode> p;
      p=path;
      p.push_back(backToDepot);
      return p;
   }



   void Vehicle::smalldump() const {
      backToDepot.dumpeval();
      std::cout << "TOTAL COST="<<cost << ", TAU= ";
      tau(); std::cout<<"\n";
   }

   void Vehicle::tau() const {
      for (int i=0; i< path.size(); i++)
         std::cout<<getnid(i)<<" , ";
   }



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
           bool oldfeasable= feasable();
           bool improved=false;
           if (isdepot(i)) return false; //should never arrive here if order is inserted
           for (int j=i+1; j<path.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
               e_swapstops(i,j);
std::cout<<"\n testing  and is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
               if (feasable() and not oldfeasable)    return true;
               if (getcost()<oldcost and feasable())  return true;
                   else  e_swap(i,j);
           }
           return false;
}


void Vehicle::hillClimbOpt() {
           int i=1;
           double bestCost=getcost();
std::cout<<"\nentering hill, initialy is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
           while (i<path.size()-1) {
              if (!findImprovment(i)) {
                 i++;
              }
              else { 
std::cout<<"\nimprovement found is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
                 i=0;
              }
           }
std::cout<<"\n\texiting hill, final is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
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




   int Vehicle::getpos(const int nid) const {
          int at=0;
          while (at<path.size() and !(getnid(at)==nid))
            at++; 
          return at;
   }

   int Vehicle::getdpos(const int oid) const {
          int at=0;
          while (at<path.size() and !(isdelivery(at) and getoid(at)==oid))
            at++; 
         return at;
    }
   int Vehicle::getppos(const int oid) const {
          int at=0;
          while (at<path.size() and !(ispickup(at) and getoid(at)==oid))
            at++; 
         return at;
    }

    

/*************** path operations aka to use Vehicke as a bucket ******************/
    bool Vehicle::isEmpty() const {return path.size()==0;}
    void Vehicle::removeNode(int nid){
           for (int at=0;at<path.size();at++) {
               if (getnid(at)==nid ){
                   path.remove(at); break; //only 1 node /path
               }
           }
    }

    void Vehicle::push_back(Dpnode pathstop) { path.push_back(pathstop); }
    void Vehicle::insert(const Dpnode &pathstop,int at) { path.insert(pathstop,at); }
    void Vehicle::insertPickup(const Order &o, const int at) { insert(*o.pickup,at); }

    void Vehicle::pushPickup(const Order &o) { push_back(*o.pickup); }
    void Vehicle::pushDelivery(const Order &o) { push_back(*o.delivery); }
    void Vehicle::pushOrder( const Order &o) {
        pushPickup(o);
        pushDelivery(o);
    }
    void Vehicle::removePickup(int oid){
          for (int at=0;at<path.size();at++) {
               if (ispickup(at) and getoid(at)==oid ){
                   path.remove(at); break;
               }
         }
    }

    void Vehicle::removeDelivery(int oid){
           for (int at=0;at<path.size();at++) {
               if (isdelivery(at) and getoid(at)==oid ){
                   path.remove(at); break; //only 1 delivery per order
               }
           }
    }
    void Vehicle::removeOrder(const Order &order){ removeOrder(order.getoid()); }
    void Vehicle::removeOrder(const int oid) {
         removePickup(oid);
         removeDelivery(oid);
    }

    void Vehicle::move(int fromi,int toj) {
          if (fromi==toj) return; //nothing to move
          path.move(fromi,toj);
    }

       
    void Vehicle::swap(int i,int j){	
          if (i==j) return; //nothing to swap
          path.swap(i,j);
    }

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
/********* MOVEMENT OF PATHSTOPS WITHIN PATH  *****/

/****** removal of nodes from the path  ********/

/***  direct evaluation **/
   void Vehicle::e_remove(int at){
          if (!path.empty()) {
              path.e_remove(at,maxcapacity);
              evalLast();  
          }
    }

/****** Indirect evaluation *****/    

    void Vehicle::e_removeOrder(const Order &order){
         removeOrder(order.getoid());
    }
    
    void Vehicle::e_removeOrder(const int oid){
         removePickup(oid);
         removeDelivery(oid);
    }

/* O(n) */
    void Vehicle::e_removePickup(int oid){
          for (int at=0;at<path.size();at++) {
               if (ispickup(at) and getoid(at)==oid ){
                   e_remove(at); break; 
               }

         }
   }

/* O(n) */
    void Vehicle::e_removeDelivery(int oid){
           for (int at=0;at<path.size();at++) {
               if (isdelivery(at) and getoid(at)==oid ){
                   e_remove(at); break; //only 1 delivery per order
               }
           }
    }






bool Vehicle::isEmptyTruck() const {return path.size()==1;}




 /****** Insertion of nodes to the path  ********/

/****** Direct evaluation *****/    
    void Vehicle::e_push_back(Dpnode pathstop) {
          path.e_push_back(pathstop,maxcapacity);
          evalLast();
    }


    
    void Vehicle::e_insert(const Dpnode &pathstop,int at) {
         path.e_insert(pathstop,at,maxcapacity);
         evalLast();
    }


/****** Indirect evaluation *****/    
    void  Vehicle::e_insertPickup(const Order &o, const int at) {
        Dpnode pickup(*o.pickup);
        e_insert(pickup,at);
    }

    
    void  Vehicle::e_pushPickup(const Order &o) {
        Dpnode  pickup(*o.pickup);
        e_push_back(pickup);
    }

    
    void Vehicle::e_pushDelivery(const Order &o) {
       Dpnode delivery(*o.delivery);
       e_push_back(delivery);
    }

    
    void Vehicle::e_pushOrder( const Order &o) {
        e_pushPickup(o);
        e_pushDelivery(o);
    }

    
/****** moves between pathstops  ********/
    void Vehicle::e_move(int fromi,int toj) {
          if (fromi==toj) return; //nothing to move
          path.e_move(fromi,toj,maxcapacity);
          evalLast();
    }

       
    void Vehicle::e_swap(int i,int j){	
          if (i==j) return; //nothing to swap
          path.e_swap(i,j,maxcapacity);
          evalLast();
    }

/*indirect*/
    void Vehicle::e_swapstops(int i,int j){
          if(i>j)  std::cout<<"This is a restrictive swap, requierment: i<j\n";  
          if ( ispickup(i) and isdelivery(j) and sameorder(i,j) ) {
               std::cout<<"This is a restrictive swap, requierment: cant swap from the same order\n";
               return;
          }
          e_swap(i,j);
     }

     

/***********************   EVALUATION **************************/


   void Vehicle::evaluate() {
     path.evaluate(maxcapacity);
     evalLast();
   };

   void Vehicle::evalLast() {
      Dpnode last=path[path.size()-1];
      backToDepot.evaluate(last,maxcapacity);
      cost= w1*backToDepot.gettotDist()+ w2*backToDepot.getcvTot() + w3*backToDepot.gettwvTot();
   }






/**************************************PLOT************************************/
void Vehicle::plot(std::string file,std::string title,int carnumber){
    Twpath<Dpnode> trace=path;
    trace.push_back(backToDepot);
    
    /** cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber */
    std::stringstream convert; 
    convert << carnumber;
    std::string carnum = convert.str();
    std::string extra=file+"vehicle"+carnum ;

    Plot1<Dpnode> graph( trace ); 
    graph.setFile( file+extra+".png" );
    graph.setTitle( title+extra );
    graph.drawInit();

    for (int i=0; i<trace.size(); i++){
        if (ispickup(i))  { 
             graph.drawPoint(trace[i], 0x0000ff, 9, true);
        } else if (isdelivery(i)) {
             graph.drawPoint(trace[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(trace[i], 0xff0000, 7, true);
        }
    }
    graph.drawPath(trace,graph.makeColor(carnumber*10), 1, true);
    graph.save();
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





double Route::testVehicle(const std::deque<int>& tp) {
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
    std::deque<int>::iterator it;
    it = std::find(orders.begin(), orders.end(), oid);
    if (it != orders.end()) return false;

    Node& np = P.N[P.O[oid].pid];
    Node& nd = P.N[P.O[oid].did];
//    Node& np = P.getPickupNodeFromOrder(oid);
//    Node& nd = P.getDeliveryNodeFromOrder(oid);

    double bestTestCost = std::numeric_limits<double>::max();
    int bestPosition = -1;
    std::deque<int> newpath; // path with predecessor inserted
    std::deque<int> newpath2; // path with predecessot and successor inserted

    for (int i=0; i<path.size(); i++) {
        newpath = path;
        std::deque<int>::iterator it2;

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
        std::deque<int>::iterator it;
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




//bppos, bdpos = best pickup postition, best delivery postition
//ppos, dpos = pickup postition, delivery postition

*/
