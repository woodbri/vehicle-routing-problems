#ifndef VEHICLE_H
#define VEHICLE_H

#include <limits>
#include <vector>
#include <sstream>


#include "twpath.h"
#include "trashnode.h"
#include "twc.h"
#include "twpath.h"
#include "plot.h"
#include "move.h"
#include "basevehicle.h"


class Vehicle: public BaseVehicle {
  private:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID ;
    inline double _MAX() const { ( std::numeric_limits<double>::max() ); };
    inline double _MIN() const { ( - std::numeric_limits<double>::max() ); };

/*
*/
  protected:

  public:
    // TODO LIST 
    // insertion will not be performed 
    //  return false if TV or CV is generated 
    bool eval_insertSteadyDumps(const Trashnode &node, int at) const;

    bool e_insertIntoFeasableTruck(const Trashnode &node,int pos);
    // insertion will be performed and return false if TV or CV is generated 
    bool e_insertMoveDumps(const Trashnode &node, int at);
    bool e_insertSteadyDumps(const Trashnode &node, int at);
    bool e_insert(const Trashnode &node, int at) { return  e_insertMoveDumps(node, at); };


    // Very TIGHT insertion 
    // insertion will not be performed if 
    //      TV and CV are  generated 
    //  true- insertion was done
    //  false- not inserted 
    bool e_insertMoveDumpsTight(const Trashnode &node, int at);
    bool e_insertSteadyDumpsTight(const Trashnode &node, int at);
    bool e_insertTight(const Trashnode &node, int at) { return  e_insertMoveDumpsTight(node, at); };
    // END TODO LIST
    
    bool eval_erase(int at, double &savings) const;
    bool applyMoveINSerasePart(int nodeNid, int pos);
    bool applyMoveINSinsertPart(const Trashnode &node, int pos);
    bool applyMoveInterSw(Vehicle &otherTruck, int truckPos, int otherTruckPos);
    bool applyMoveIntraSw(int fromPos, int withPos);
    bool e_makeFeasable(int currentPos);
    long int eval_insertMoveDumps( const Trashnode &node, std::deque<Move> &moves, int fromTruck, int formPos, int toTruck, double savings, double factor ) const;
    long int eval_intraSwapMoveDumps( std::deque<Move> &moves, int  truckPos, int fromPos,  double factor) const ;
    long int eval_interSwapMoveDumps( std::deque<Move> &moves, const Vehicle &otherTruck,int  truckPos,int  otherTruckPos, int fromPos,  double factor) const;
    bool e_insertDumpInPath( const Trashnode &going );
    bool deltaTimeGeneratesTV(const Trashnode &dump, const Trashnode &node) const; 
    bool deltaCargoGeneratesCV(const Trashnode &node, int pos) const;
    bool deltaCargoGeneratesCV_AUTO(const Trashnode &node, int pos) const;
    bool deltaTimeGeneratesTV(const Trashnode &node,int pos) const;
    bool deltaTimeGeneratesTV_AUTO(const Trashnode &node,int pos) const;
    //--------------------------------------------------------------------
    // constructors
    //--------------------------------------------------------------------

    Vehicle() {
        maxcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
    };

    Vehicle(std::string line,const Bucket &otherlocs): BaseVehicle(line,otherlocs)   { }

	//for cost function
	Trashnode C , last;
	double ttSC, ttDC, ttCD, ttDE, ttCC;
	double realttSC, realttDC, realttCD, realttDE, realttCC;
	int N, Nreal,minDumpVisits,maxDumpVisits,realDumpVisits;
        int Z,z1,z2,realz1,realz2,n,z,Zmissing;
	double arrivalEcloseslast,realArrivalEclosesLast,shiftLength;
	double serviceE;
	double totalTime,realTotalTime;
	double forcedWaitTime,totalWaitTime,idleTime;
	double realForcedWaitTime,realtotalWaitTime,realIdleTime;
	double idleTimeSCDE,idleTimeSDCDE;
	double realIdleTimeSCDE,realIdleTimeSDCDE;
	inline int  realN() { return ( path.getDumpVisits() + 1 ) ;}
	inline double  totalServiceTime() { 
		return ( path.getTotServiceTime()+  dumpSite.getservicetime() + endingSite.getservicetime() ) ;}  
        void setInitialValues( const Trashnode &node,const TWC<Trashnode> &twc, const Bucket &picks){
		C = node;
		ttSC=twc.getAverageTime(depot,picks);
		ttDC=twc.getAverageTime(dumpSite,picks);
		ttCD=twc.getAverageTime(picks,dumpSite);
		ttDE=twc.travelTime(dumpSite,endingSite);
		ttCC=twc.travelTime(C,C);
		serviceE=endingSite.getservicetime();
                shiftLength=endTime-startTime;
		e_makeFeasable(0);
		Z= floor( maxcapacity/C.getdemand());
		arrivalEcloseslast = C.closes() + ttCD + dumpSite.getservicetime() + ttDE;
		totalTime=0;
		
		N =-1; 
		do {
		N++;
		totalTime=depot.getservicetime()  + ttSC + ttDE + endingSite.getservicetime() 
			+ N * Z * C.getservicetime() + N * (Z - 1) * ttCC + (N -1) * ttDC
			+ N * ( dumpSite.getservicetime() + ttCD );
		}
		while ( totalTime < arrivalEcloseslast + serviceE );
		forcedWaitTime=endTime-( arrivalEcloseslast  +  serviceE );
                totalWaitTime=endTime- (endingSite.getArrivalTime() +serviceE);
		idleTimeSCDE= C.closes() - (depot.getservicetime() + ttSC );
		z1 = idleTimeSCDE / (C.getservicetime() + ttCC);
		idleTimeSDCDE= C.closes() - (dumpSite.getDepartureTime() + ttDC );
		z2 = idleTimeSDCDE / (C.getservicetime() + ttCC);
                idleTime= totalWaitTime-forcedWaitTime;

		last=(size()>1)? path[size()-1] : C ;
		realttSC=path.size()>1?path[1].getTotTravelTime()  :ttSC;
		realttCC=size()>1?(path.getTotTravelTime()-realttSC)/(size()-1) :ttCC;
		realttCD=size()>1? twc.travelTime( path[path.size()-1] , dumpSite )  :ttCD;
                realttDC=0;
		if ( path.getDumpVisits() ) {
			for (int i=1;i<path.size()-1;i++) 
			    if (path[i].isdump()) realttDC+= twc.travelTime( path[i], path[i+1]);
                        realttDC/path.getDumpVisits();
                } else realttDC=ttDC;
		realttDE=ttDE;
		realArrivalEclosesLast = (size()>1)? path[size()-1].closes() + realttCD + dumpSite.getservicetime() + realttDE : arrivalEcloseslast;
		realForcedWaitTime=endTime-( realArrivalEclosesLast  +  serviceE );
		realTotalTime= endingSite.getArrivalTime();
		realIdleTime =  realArrivalEclosesLast -  realTotalTime ;
		n  = size() - 1 - ( realN() - 1 );
	        z = (realN()==1)?  n  : n % Z ;
		Zmissing=(Z>z)? Z-z:0;
		realIdleTimeSCDE =  ( z )? realIdleTime - (C.getservicetime() + realttCC ) * Zmissing :
                		           C.closes() - ( depot.getDepartureTime() +  realttSC);
		realz1 = std::min ( (int) (floor(realIdleTime / ( C.getservicetime() + realttCC ) )) ,Zmissing ) ;
		realIdleTimeSDCDE =  C.closes() - ( dumpSite.getDepartureTime() +  realttDC );
		realz2 = realIdleTimeSDCDE / ( C.getservicetime() +  realttCC );

       		
		double deltaTime= totalTime - realTotalTime;
dumpCostValues();
	}

void dumpCostValues(){
	std::cout<<" +++++++++++++++++++++  	 TRUCK #<<"<<vid<<"      +++++++++++++++++++++ \n\n\n"; 
	std::cout<<" Average Container \t"; 
	C.dump();
	std::cout<<" ------  truck time limits -------\n"
		<<"Shift Starts\t"<<startTime<<"\n"
		<<"Shift ends\t"<<endTime<<"\n"
		<<"Shift length\t"<<shiftLength<<"\n";

	std::cout<<" ------  current path -------\n";
	tau();
		
		
	std::cout<<"\n\n\n ------Real  Values of current truck in the solution -------\n"
                <<"realttSC=\t"     <<realttSC<<"\n"
                <<"realttCC=\t"     <<realttCC<<"\n"
                <<"realttCD=\t"     <<realttCD<<"\n"
                <<"realttDC=\t"     <<realttDC<<"\n"
                <<"realttDE=\t"     <<realttDE<<"\n"
		<<"service(E)\t"<<serviceE<<"\n"
		<<" Z = floor( maxcapacity/C.getdemand() )\n"
		<<Z<<" = floor( "<<maxcapacity<<"/"<<C.getdemand()<<" )\n"

                <<" \nrealarrivalEcloseslast = path[size()-1].closes() + realttCD + dumpSite.getservicetime() + realttDE \n"
                << arrivalEcloseslast<< " = " <<path[size()-1].closes()<<" + "<<realttCD<<" + "<<dumpSite.getservicetime()<<" + "<<realttDE<< "\n"

                <<"\n realForcedWaitTime = shiftEnds -( realArrivalEclosesLast  +  serviceE )\n"
                <<realForcedWaitTime<<" = "<<endTime<<" - (" <<realArrivalEclosesLast <<" + " <<serviceE<<" )\n"

		<<" \nrealN \n"
		<<realN()<<"\n"

		<<" realTotalTime =  endingSite.getArrivalTime()\n"
		<< realTotalTime<<" = "<<  endingSite.getArrivalTime()<<"\n"

		<<" n = size() - 1 - ( realN()-1 )   <-- total number of containers served\n"
		<< n <<" = "<<size()<<" - 1 - (" <<realN()<<" -1 )\n"

	        <<"  z = (realN()==1)  z = n  : n % Z   <-- total number of containers in last trip\n"
	        <<  z <<" = ( "<<realN()<<" == 1 )" << z<<" = "<<n <<" : "<< n<< " % " <<Z<<"\n" 
		
		<<"realIdleTime =  realArrivalEclosesLast -  realTotalTime\n" 
		<<realIdleTime<<" = "<< realArrivalEclosesLast<<" -  "<<realTotalTime<<"\n" 

		<<"Zmissing=(Z>z)? Z-z:0\n"
		<<Zmissing<<" = "<<Z<<" > "<<z<<")?"<<Z<<" - "<<z<<":0\n";

		std::cout<<"\n realz1 = min ( realIdleTimeSCDE / (C.getservicetime() + realttCC) , Zmissing )\n"
		<<realz1<<" = min (" <<realIdleTimeSCDE<<" / ( "<<C.getservicetime()<<" + "<< realttCC<<" ) ,"<<Zmissing<<"\n"
		<<realz1<<" containers can be served in a trip: SCDE\n"

		<<" \n realIdleTimeSDCDE = C.closes() - ( dumpSite.getDepartureTime() + realttDC)\n"
		<<realIdleTimeSDCDE<<" = "<< C.closes()<<" - ( "<<dumpSite.getDepartureTime()<<" + "<< realttDC<<" )\n"
	
		<<"\n realz2 = idleTimeSDCDE / (C.getservicetime() + ttCC)\n"
		<<realz2<<" = " <<realIdleTimeSDCDE<<" / ( "<<C.getservicetime()<<" + "<< realttCC<<" )\n"
		<<realz2<<" containers can be served in a trip: SDCDE\n"


;


        std::cout<<"\n\n\n ------estimated  Values for emtpy truck that is in the solution -------\n"
		<<"ttSC=\t"	<<ttSC<<"\n" 
		<<"ttCC=\t"	<<ttCC<<"\n" 
		<<"ttCD=\t"	<<ttCD<<"\n" 
		<<"ttDC=\t"	<<ttDC<<"\n" 
		<<"ttDE=\t"	<<ttDE<<"\n" 
                <<"service(E)\t"<<serviceE<<"\n"
                <<" Z = floor( maxcapacity/C.getdemand() )    <<--- this is still the estimation\n"
                <<Z<<" = floor( "<<maxcapacity<<"/"<<C.getdemand()<<" )\n"

		<<" \narrivalEcloseslast = C.closes() + ttCD + dumpSite.getservicetime() + ttDE \n"
		<< arrivalEcloseslast<< " = " <<C.closes()<<" + "<<ttCD<<" + "<<dumpSite.getservicetime()<<" + "<<ttDE<< "\n"

		<<"\n forcedWaitTime = shiftEnds -( arrivalEcloseslast  +  serviceE )\n"
		<<forcedWaitTime<<" = "<<endTime<<" - (" <<arrivalEcloseslast <<" + " <<serviceE<<" )\n"

                <<" \nwith N=1\n"
                <<" upperLimit(totalTime) = depot.getservicetime()  + ttSC + ttDE + endingSite.getservicetime()\n"
                        <<"+ N * Z * C.getservicetime() + N * (Z - 1) * ttCC + (N -1) * ttDC\n"
                        <<"+ N * ( dumpSite.getservicetime() + ttCD )\n"
                << totalTime<<" = "<< depot.getservicetime()<< " + "<< ttSC<<" + "<<ttDE<<" + "<< endingSite.getservicetime()<<"\n"
                        <<" + "<< N<<" *"<< Z<< " * "<< C.getservicetime()<<" +"<< N<<" * ("<<Z<<" - 1) * "<<ttCC<<" + ("<<N<<" -1) *"<< ttDC<<"\n"
                        <<" + "<< N<<" * (" <<dumpSite.getservicetime()<<" +"<< ttCD<<" )"<<"\n"

                <<" \n last (and only trip) can/cant serve Z containers? =  upperLimit(totalTime) <= arrivalEcloseslast (CAN) \n"
                <<" last (and only trip) " <<( totalTime <= arrivalEcloseslast ?"CAN":"CAN NOT" )<< " serve <<"<<Z<<" containers  "
                        << (totalTime <= arrivalEcloseslast) <<"="<<totalTime<<" <= " <<arrivalEcloseslast <<"\n"

                <<" \n idleTimeSCDE = C.closes() - ( depot.getDepartureTime() + ttSC )\n"
                <<idleTimeSCDE<<" = "<< C.closes()<<" - ( "<<depot.getDepartureTime()<<" + "<< ttSC<<" )\n"

                <<"\n z1 = idleTimeSCDE / (C.getservicetime() + ttCC)\n"
                <<z1<<" = " <<idleTimeSCDE<<" / ( "<<C.getservicetime()<<" + "<< ttCC<<" )\n"
                <<z1<<" containers can be served in a trip: SCDE\n"

                <<" \n idleTimeSDCDE = C.closes() - ( dumpSite.getDepartureTime() + ttDC)\n"
                <<idleTimeSDCDE<<" = "<< C.closes()<<" - ( "<<dumpSite.getDepartureTime()<<" + "<< ttDC<<" )\n"

                <<"\n z2 = idleTimeSDCDE / (C.getservicetime() + ttCC)\n"
                <<z2<<" = " <<idleTimeSDCDE<<" / ( "<<C.getservicetime()<<" + "<< ttCC<<" )\n"
                <<z2<<" containers can be served in a trip: SDCDE\n"




;
		std::cout<<"\n\n\n ------  DOCUMENT COST  VARIABLES -------" <<" ------  REAL COST  VARIABLES -------\t "<<" ------  PERCENTAGES  -------\n"
			<<"ttSC=\t"	<<ttSC<<"\t" <<"realttSC=\t"	<<realttSC<<"\t" <<"realttSC/ttSC=\t\t"	<<realttSC/ttSC*100<<"%\n"
			<<"ttCC=\t"	<<ttCC<<"\t" <<"realttCC=\t"	<<realttCC<<"\t" <<"realttCC/ttCC=\t\t"	<<realttCC/ttCC*100<<"%\n"
			<<"ttCD=\t"	<<ttCD<<"\t" <<"realttCD=\t"	<<realttCD<<"\t" <<"realttCD/ttCD=\t\t"	<<realttCD/ttCD*100<<"%\n"
			<<"ttDC=\t"	<<ttDC<<"\t" <<"realttDC=\t"	<<realttDC<<"\t" <<"realttDC/ttDC=\t\t"	<<realttDC/ttDC*100<<"%\n"
			<<"ttDE=\t"	<<ttDE<<"\t" <<"realttDE=\t"	<<realttDE<<"\t" <<"realttDE/ttDE=\t\t"	<<realttDE/ttDE*100<<"%\n"
			<<"service(E)\t"<<serviceE<<"\t" <<"service(E)\t"<<serviceE<<"\n"
			<<"Z\t\t"	<<Z<<"\n"
			<<"N\t\t"	<<N<<"\t" <<"real N\t"<<realN()<<"\t" <<"realN/N \t\t"<<realN()/N*100<<"%\n"
			<<"totalTime\t"	<<totalTime<<"\t" <<"realTotalTime\t"<<realTotalTime<<"\t" <<"realTotalTime/totalTime\t"<<realTotalTime/totalTime*100<<"%\n"
			                                                         <<"\t\t\t\t\t\t\t"<<"realTotalTime/shiftLength\t"<<realTotalTime/shiftLength*100<<"%\n"
			<<"totalWaitTime\t"<<totalWaitTime<<"\t\t\t\ttotalWaitTime/totalTime\t"<<totalWaitTime/totalTime*100<<"%\n"
                                        <<"\t\t\t\t\t\t\t"<<"totalWaitTime/shiftLength\t"<<totalWaitTime/shiftLength*100<<"%\n"

			<<"forcedWaitTime\t"<<forcedWaitTime<<"\t\t\t\t" <<"forcedWaitTime/shiftLength\t"<<forcedWaitTime/shiftLength*100<<"%\n"
			<<"idleTime\t"<<idleTime<<"\t"<<"idleTime/shiftLength\t"<<idleTime/shiftLength*100<<"%\n"
			<<"arrivalEcloseslast\t"<<arrivalEcloseslast<<"\n";

	};
};


#endif

