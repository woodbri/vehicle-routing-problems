
//#include <stdexcept>
//#include <algorithm>
//#include <iostream>
//#include <fstream>
#include <sstream>
//#include <string>
//#include <vector>
#include <list>
//#include <math.h>

//#include "order.h"
//#include "node.h"
#include "pathnode.h"
#include "path.h"



   int Path::getdpos(const int oid) const {
/*          int i=0;
          while (i<path.size() and !(path[i].isdelivery() and path[i].getoid()==oid))
            i++; 
         return i;
*/    }
   int Path::getppos(const int oid) const {
/*          int i=0;
          while (i<path.size() and !(path[i].ispickup() and path[i].getoid()==oid))
            i++; 
         return i;
*/    }



    void Path::removeOrder(const int oid){
         removePickup(oid);
         removeDelivery(oid);
    }

    void Path::removePickup(int oid){
/*          for (int i=0;i<path.size();i++) {
               if (path[i].ispickup() and path[i].getoid()==oid ){
                   remove(i); break; //only 1 pickup per order
               }

         }
*/    }

    void Path::removeDelivery(int oid){
/*           for (int i=0;i<path.size();i++) {
               if (path[i].isdelivery() and path[i].getoid()==oid ){
                   remove(i); break; //only 1 delivery per order
               }
           }
*/    }      

    void Path::remove(int at){ 
          if (!path.empty()) path.erase(path.begin()+at);
    }
 
    double Path::getcost(double w1,double w2,double w3) { 
        setvalues(0);
        return   w1*D + w2*TWV + w3*CV;
    }


    void Path::push_back(pathNode pathstop) {
          path.push_back(pathstop);
    }

    void Path::insert(pathNode pathstop,int at) {
         path.insert(path.begin()+at,pathstop);
    }

    void Path::setvalues(int at){
/*         if (at<path.size()) {
              if (at==0) path[at].setValues(*depot);
              else path[at].setValues(path[at-1]);
              setvalues(at+1);
         } else {
              setDepotValues();
         };
*/     }


    void Path::move(int fromi,int toj) {
              //some checking migh go in route level
              if (fromi<toj){
                  insert(path[fromi],toj+1);
                  remove(fromi);
              }
               else {
                  insert(path[fromi],toj);
                  remove(fromi+1);
              }
    }

     void Path::swapnodes(int i,int j){
/*          if(i>j)  std::cout<<"This is a restrictive swap, requierment: i<j\n";  
          else if (ispickup(i) and isdelivery(j) and sameorder(i,j)) std::cout<<"This is a restrictive swap, requierment: cant swap from the same order\n";
          else {
              pathNode temp(path[i]);
              path[i]=path[j];
              path[j]=temp;
  //            setvalues(i); //update values starting from i
  //            setvalues(0);
          }
**/     }

     void Path::swap(int i,int j){
          pathNode temp(path[i]);
          path[i]=path[j];
          path[j]=temp;
    //      if (i<j) setvalues(i);
    //      else setvalues(j);
    //      setvalues(0);
     }
                 

     void Path::setDepotValues() {
              int at= path.size()-1;
              //D = path[at].totDistFromDepot+depot->distance(path[at].getnode());
              //D = path[at].totDistFromDepot+depot->distance(path[at]);
              //wv_depot=depot->lateArrival(D);
              cv_depot=path[at].cv;
              TWV = path[at].twvTot;
              CV = path[at].cvTot;
              TWV = (twv_depot)? TWV+1:TWV;
              CV = (cv_depot)? CV+1:CV;
      }


void Path::dump()  {
    setvalues(0);
     for (int i=0;i<path.size();i++){
          std::cout<<"\npath stop #:"<<i<<"\n";
          path[i].dump();
     };
}     



void Path::smalldump() {
    setvalues(0);
    std::cout << "D="<<D << ", "
              << "TWV="<<TWV << ", "
              << "CV=" <<CV<< ", ";
    if(twv_depot) std::cout<<"depot: has twv ";
    if(cv_depot) std::cout<<"depot: has cv ";
    std::cout << "\nPath(nid,oid): [";
    for (int i=0; i<path.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << "("<<getnid(i)/*<<","<<getoid(i)<<")"*/;
    }
    std::cout << "]\n";
}
