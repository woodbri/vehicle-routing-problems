#include <iostream>
#include <deque>


//#include "order.h"
//#include "orders.h"
#include "twpath.h"
#include "twc.h"
#include "plot.h"
#include "vehicle.h"

#include <sstream>


//antes del from no lo voy a poner
int   Vehicle::getPosLowLimit(int nid ,int from ,const TWC<Trashnode> &twc )const {
       int RL;
       for (RL=size(); RL>from  and twc.isCompatibleIJ( nid , path[RL-1].getnid() );RL--) {};
std::cout<<"RL"<<RL<<"\t";
       return RL;
}

int   Vehicle::getPosHighLimit(int nid ,int from, int to, const TWC<Trashnode> &twc) const {
//despues del to no lo voy a poner
       int LR;
       for (LR=from ; LR<to and twc.isCompatibleIJ( path[LR].getnid(), nid ) ; LR++) {};
std::cout<<"LR"<<LR<<"\n";
       return LR;
}
       
// state
   bool Vehicle::isvalid() const {
       //What is the validity of a truck?
       return true;   
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
      std::cout<<" (";
      for (int i=0; i< path.size(); i++)
         std::cout<<getid(i)<<" , ";
      std::cout<<" )";
   }



/* to forget about evaluation everything done in Vehicle has to be evaluated */
    
    
    
    
/********* MOVEMENT OF PATHSTOPS WITHIN PATH  *****/

/****** removal of nodes from the path  ********/

/***  direct evaluation **/






bool Vehicle::isEmptyTruck() const {return path.size()==1;}





/****** Direct evaluation *****/    
    void Vehicle::e_push_back(const Trashnode &pathstop) {
          path.e_push_back(pathstop,maxcapacity);
          evalLast();
    }

   void Vehicle::e_erase(int at){
          if (!path.empty()) {
              path.e_remove(at,maxcapacity);
              evalLast();  
          }
    }

    void Vehicle::e_insert(const Trashnode &pathstop,int at) {
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

     

/***********************   EVALUATION **************************/


   void Vehicle::evaluate(int from) {
     assert(from < size());
     path.evaluate(from, maxcapacity);
     evalLast();
   };

   void Vehicle::evalLast() {
      Trashnode last=path[path.size()-1];
      backToDepot.evaluate(last,maxcapacity);
      cost= w1*backToDepot.gettotDist()+ w2*backToDepot.getcvTot() + w3*backToDepot.gettwvTot();
   }





/**************************************PLOT************************************/
void Vehicle::plot(std::string file,std::string title,int carnumber){
//std::cout<<"USING VEHICLE PLOT\n";
    Twpath<Trashnode> trace=path;
    trace.push_back(backToDepot);
    
    /** cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber */
    std::stringstream convert; 
    convert << carnumber;
    std::string carnum = convert.str();
    std::string extra=file+"vehicle"+carnum ;

    Plot<Trashnode> graph( trace ); 
    graph.setFile( file+extra+".png" );
    graph.setTitle( title+extra );
    graph.drawInit();
    for (int i=0; i<size(); i++){
        if (ispickup(i))  { 
             graph.drawPoint(path[i], 0x0000ff, 9, true);
        } else if (isdump(i)) {
             graph.drawPoint(path[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(path[i], 0xff0000, 7, true);
        }
    }
    plot(graph,carnumber);
    graph.save();
}

void Vehicle::plot(Plot<Trashnode> graph, int carnumber){
//std::cout<<"USING VEHICLE PLOT  1\n";
    Twpath<Trashnode> trace=path;
    trace.push_back(backToDepot);
    graph.drawPath(trace,graph.makeColor(carnumber*10), 1, true);
}



std::deque<int> Vehicle::getpath() const {
    std::deque<int> p=path.getpath();
    p.push_back(backToDepot.getnid());
    return p;
}

