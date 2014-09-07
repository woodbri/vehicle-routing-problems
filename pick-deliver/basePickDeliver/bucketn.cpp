#include <iostream>
#include <deque>


#include "order.h"
#include "twpath.h"
#include "vehicle.h"
#include "bucketn.h"
#include "compatible.h"
#include "plot1.h"

#include <sstream>





/***************************** DUMP PRINTS PLOTS   ********/

   void BucketN::dump() const {
     for (int i=0;i<path.size();i++){
         std::cout<<"\npath stop #:"<<i<<"\n";
          path[i].dumpeval();
     }
   }

   Twpath<Dpnode>& BucketN::Path() { return path; }
   Twpath<Dpnode> BucketN::getpath() const  { return path; }



   void BucketN::smalldump() const { tau(); std::cout<<"\n"; }

   void BucketN::tau() const {
      for (int i=0; i< path.size(); i++)
         std::cout<<getnid(i)<<" , ";
   }

/* functions to find position */
   bool BucketN::in(int nid) const {
       for (int i=0; i< path.size(); i++)
           if (nid==getnid(i)) return true;
       return false;
   }

 
   int BucketN::pos(int nid) const {
       for (int i=0; i< path.size(); i++)
           if (nid==getnid(i)) return i;
       return -1;
   }

   bool BucketN::in(const Dpnode &node) const { return in(node.getnid()); }
   bool BucketN::in(const Order &o) const { return in( o.getdid());}
   int BucketN::pos(const Dpnode &node) const { return pos(node.getnid()); }

   int BucketN::getdpos(const int oid) const {
          int at=0;
          while (at<path.size() and !(isdelivery(at) and getoid(at)==oid))
            at++; 
         return at;
    }

   int BucketN::getppos(const int oid) const {
          int at=0;
          while (at<path.size() and !(ispickup(at) and getoid(at)==oid))
            at++; 
         return at;
    }
   int BucketN::getppos(const Order &o) const { return pos( o.getpid());}
   int BucketN::getdpos(const Order &o) const { return pos( o.getdid());}
    

/*************** path operations aka to use Vehicke as a bucket ******************/
    bool BucketN::empty() const {return path.empty();}
    void BucketN::removeNode(int nid){
           for (int at=0;at<path.size();at++) {
               if (getnid(at)==nid ){
                   path.remove(at); break; //only 1 node /path
               }
           }
    }

    void BucketN::push_back(Dpnode pathstop) { path.push_back(pathstop); }
    void BucketN::insert(const Dpnode &pathstop,int at) { path.insert(pathstop,at); }
    void BucketN::insertPickup(const Order &o, const int at) { insert(*o.pickup,at); }

    void BucketN::pushPickup(const Order &o) { push_back(*o.pickup); }
    void BucketN::pushDelivery(const Order &o) { push_back(*o.delivery); }
    void BucketN::pushOrder( const Order &o) {
        pushPickup(o);
        pushDelivery(o);
    }
    void BucketN::removePickup(int oid){
          for (int at=0;at<path.size();at++) {
               if (ispickup(at) and getoid(at)==oid ){
                   path.remove(at); break;
               }
         }
    }

    void BucketN::removeDelivery(int oid){
           for (int at=0;at<path.size();at++) {
               if (isdelivery(at) and getoid(at)==oid ){
                   path.remove(at); break; //only 1 delivery per order
               }
           }
    }
    void BucketN::removeOrder(const Order &order){ removeOrder(order.getoid()); }
    void BucketN::removeOrder(const int oid) {
         removePickup(oid);
         removeDelivery(oid);
    }

    void BucketN::move(int fromi,int toj) {
          if (fromi==toj) return; //nothing to move
          path.move(fromi,toj);
    }

       
    void BucketN::swap(int i,int j){	
          if (i==j) return; //nothing to swap
          path.swap(i,j);
    }

    
void BucketN::plot(std::string file,std::string title,int carnumber){
    Twpath<Dpnode> trace=path;
std::cout<<"USING BUCKET PLOT\n"; 
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


