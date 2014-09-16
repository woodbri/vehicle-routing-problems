#include <iostream>
#include <deque>


#include "order.h"
#include "orders.h"
#include "twpath.h"
#include "vehicle.h"
#include "compatible.h"
#include "plot.h"

#include <sstream>


double Vehicle::testInsertPUSH(const Order &order, const Orders &orders, int &pickPos, int &delPos,const Compatible &twc) {
       //lastDelivery 
       //lastPickup
       //check if its D's -> O
       int oid=order.getoid();
       int route_oid;
       bool flag=true;
std::cout<<"lastPick "<<lastPickup<<"\t lastDelivery "<<lastDelivery<<"\n";
       if (lastPickup==lastDelivery) {
std::cout<<"test Insert 1\n";
           pickPos=1;delPos=2;
           return true;
       }
       for (int i=lastPickup;i<lastDelivery;i++) {  //deliveries intermedios -> orden
           route_oid=getoid(i);
std::cout<<"test Insert 2---"<<route_oid<<" vs "<<oid<<"\n";
std::cout<<"compat "<<(orders.isPUSHcompat(route_oid,oid)?"yes":"no")<<"\n";


           flag=flag and orders.isPUSHcompat(route_oid,oid);
       }
       if (flag==false) { pickPos=-1;delPos=-1; return std::numeric_limits<double>::max(); };
std::cout<<"test Insert 3---\n";
       int pLR,pRL,dLR,dRL;
       Dpnode pickup=order.getPickup();
       Dpnode delivery=order.getDelivery();

       for (pLR=lastDelivery;pLR<size() and twc.isCompatibleIJ( pickup.getnid(),path[pLR].getnid());pLR++) {};
       for (pRL=size()-1;pRL>lastDelivery and twc.isCompatibleIJ(path[pLR].getnid() , pickup.getnid() );pLR++) {};
std::cout<<"pLR"<<pLR<<"\t";
std::cout<<"pRL"<<pRL<<"\n";

       for (dLR=lastDelivery;dLR<size() and twc.isCompatibleIJ(delivery.getnid(),path[dLR].getnid());dLR++) {};
       for (dRL=size()-1;dRL>lastDelivery and twc.isCompatibleIJ(path[dRL].getnid(),delivery.getnid());dLR++) {};
std::cout<<"dLR"<<dLR<<"\t";
std::cout<<"dRL"<<dRL<<"\n";

           
       return flag;
}
           
 


// *******
bool Vehicle::insertOrderAfterLastPickup(const Order &order,const Compatible &twc){
    int i;
    for (i=size()-1; i>=0 and not ispickup(i); i--) {};
    int lastPickPos=i+1;

    int deliverPosLR=0;
    int pickPosLR=0;
    int deliverPosRL=size()-1;
    int pickPosRL=size()-1;
    int Px= order.getpid();
    int Dx= order.getdid();
    for  (int i=0; i<size(); i++) {
        pickPosLR=i;
        if ( twc.isCompatibleIJ(getnid(i),Px) ) continue;
        else {break; };
    };

    for (int i=size()-1; i>=0;i--) {
        pickPosRL=i;
        if ( twc.isCompatibleIJ(Px,getnid(i)) ) continue;
        else {break; };
    };

    if (pickPosRL>pickPosLR) std::cout<<" SOMETHING WENT WRONG 1 \n";
    else std::cout<<"\nCan place the picup from pick Pos RL="<<++pickPosRL<<"\t up to pick Pos LR="<<pickPosLR<<"\n";

    for  (int i=0; i<size();i++) {
        deliverPosLR=i;
        if ( twc.isCompatibleIJ(getnid(i),Dx) ) continue;
        else {break;};
    };
    for (int i=size()-1; i>=0;i--) {
        deliverPosRL=i;
        if ( twc.isCompatibleIJ(Dx,getnid(i)) ) continue;
        else {break; };
    };
    // teoricamente Rl<LR
    if (deliverPosLR<deliverPosRL) std::cout<<" SOMETHING WENT WRONG 2";
std::cout<<"\n can place the deliver from Pos RL="<<++deliverPosRL<<"\t up to deliver Pos LR="<<deliverPosLR<<"\n";
    double bestCost= std::numeric_limits<double>::max();
    int bestI,bestJ;
    for (int i=pickPosRL; i<=pickPosLR+1;i++) {
std::cout<<"CYCLE"<<i<<"\n";
        insert(order.getPickup(),i);
        insert(order.getDelivery(),i+1);
        for (int j=i+1; j<=deliverPosLR+2;j++) {
std::cout<<"        CYCLE"<<j<<"\n";
tau(); std::cout<<"\n";
            if (getcost()<bestCost) {
                bestCost=getcost();
                bestI=i; bestJ=j;
            }
            swap(j,j+1);
        };
tau(); std::cout<<"\n";
        e_erase(size()-1);
        e_erase(i);
tau(); std::cout<<"\n";
    }
           
tau(); std::cout<<"\n";
    insert(order.getPickup(),bestI);
tau(); std::cout<<"\n";
    insert(order.getDelivery(),bestJ);
dump(); std::cout<<"<---- salgo con\n"<<(feasable()? "FEASABLE":"NOT FEASABLE");
    return feasable();
}



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
               swapstops(i,j);
std::cout<<"\n testing  and is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
               if (feasable() and not oldfeasable)    return true;
               if (getcost()<oldcost and feasable())  return true;
                   else  swap(i,j);
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


/* to forget about evaluation everything done in Vehicle has to be evaluated */
    
    
    
    
/********* MOVEMENT OF PATHSTOPS WITHIN PATH  *****/

/****** removal of nodes from the path  ********/

/***  direct evaluation **/
/****** Indirect evaluation *****/    

    void Vehicle::removeOrder(const Order &order){
         removeOrder(order.getoid());
    }
    
    void Vehicle::removeOrder(const int oid){
         removePickup(oid);
         removeDelivery(oid);
    }

    int Vehicle::pos(int nid) const{return path.pos(nid);}
             
    bool Vehicle::in(int nid) const{
         return ( (path.pos(nid)!=-1) ? true:false );
    }
              

/* O(n) */
    void Vehicle::removePickup(int oid){
          for (int at=0;at<path.size();at++) {
               if (ispickup(at) and getoid(at)==oid ){
                   e_erase(at); break; 
               }
         }
   }

/* O(n) */
    void Vehicle::removeDelivery(int oid){
           for (int at=0;at<path.size();at++) {
               if (isdelivery(at) and getoid(at)==oid ){
                   e_erase(at); break; //only 1 delivery per order
               }
           }
    }






bool Vehicle::isEmptyTruck() const {return path.size()==1;}





/****** Direct evaluation *****/    
    void Vehicle::e_push_back(const Dpnode &pathstop) {
          path.e_push_back(pathstop,maxcapacity);
          evalLast();
    }

   void Vehicle::e_erase(int at){
          if (!path.empty()) {
              path.e_remove(at,maxcapacity);
              evalLast();  
          }
    }

    void Vehicle::e_insert(const Dpnode &pathstop,int at) {
         path.e_insert(pathstop,at,maxcapacity);
         evalLast();
    }

    void Vehicle::e_move(int fromi,int toj) {
          if (fromi==toj) return; //nothing to move
          path.e_move(fromi,toj,maxcapacity);
          evalLast();
    }

    void Vehicle::e_clean() {
          path.resize(1);
          evalLast();
    }

       
    void Vehicle::e_swap(int i,int j){	
          if (i==j) return; //nothing to swap
          path.e_swap(i,j,maxcapacity);
          evalLast();
    }

/****** Indirect evaluation *****/    
    void  Vehicle::insertPickup   (const Order &o, const int at)  { e_insert(*o.pickup,at); }
    void  Vehicle::insertDelivery (const Order &o, const int at) { e_insert(*o.delivery,at); }

    
    void  Vehicle::pushPickup(const Order &o)  { e_push_back(*o.pickup); }
    void Vehicle::pushDelivery(const Order &o) { e_push_back(*o.delivery); }

    
    void Vehicle::pushOrder( const Order &o) {
        pushPickup(o);
        pushDelivery(o);
        lastPickup=size()-2;
        lastDelivery=size()-1;
    }

    
/****** moves between pathstops  ********/

    void Vehicle::swapstops(int i,int j){
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
std::cout<<"USING VEHICLE PLOT\n";
    Twpath<Dpnode> trace=path;
    trace.push_back(backToDepot);
    
    /** cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber */
    std::stringstream convert; 
    convert << carnumber;
    std::string carnum = convert.str();
    std::string extra=file+"vehicle"+carnum ;

    Plot<Dpnode> graph( trace ); 
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




Twpath<Dpnode> Vehicle::getpath() const {
    Twpath<Dpnode> p=path;
    p.push_back(backToDepot);
    return p;
}
