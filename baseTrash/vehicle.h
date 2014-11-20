/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
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

  protected:
    typedef  TwBucket<Trashnode> Bucket;
    typedef  unsigned long int UID ;
    typedef  unsigned long int POS ;
    inline double _MAX() const { ( std::numeric_limits<double>::max() ); };
    inline double _MIN() const { ( - std::numeric_limits<double>::max() ); };
    typedef std::set<Move,Move::compMove> Moves;

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
    
    bool applyMoveINSerasePart(int nodeNid, int pos);
    bool applyMoveINSinsertPart(const Trashnode &node, int pos);
    bool applyMoveInterSw(Vehicle &otherTruck, int truckPos, int otherTruckPos);
    bool applyMoveIntraSw(int fromPos, int withPos);
    bool e_makeFeasable(int currentPos);
    long int eval_insertMoveDumps( const Trashnode &node, std::deque<Move> &moves, int fromTruck, int formPos, int toTruck, double savings, double factor ) const;
    long int eval_intraSwapMoveDumps( std::deque<Move> &moves, int  truckPos, int fromPos,  double factor) const ;
    //long int eval_interSwapMoveDumps( std::deque<Move> &moves, const Vehicle &otherTruck,int  truckPos,int  otherTruckPos, int fromPos,  double factor) const;
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


    Vehicle(int _vid, int _start_id, int _dump_id, int _end_id,
            int _capacity, int _dumpservicetime, int _starttime,
            int _endtime, const Bucket &otherlocs)
        : BaseVehicle(_vid, _start_id, _dump_id, _end_id,
                      _capacity, _dumpservicetime, _starttime,
                      _endtime, otherlocs) {};


    double timePCN(POS prev, POS curr, POS next) const;
    double timePCN(Trashnode &prev, Trashnode &curr, Trashnode &next) const;

    long int eval_intraSwapMoveDumps( Moves &moves, int  truckPos,  double factor ) const;
    long int eval_interSwapMoveDumps( Moves &moves, const Vehicle &otherTruck,int  truckPos,int  otherTruckPos, double factor   ) const; 
    long int eval_insertMoveDumps( const Trashnode &node, Moves &moves, int fromTruck, int formPos, int toTruck, double savings, double factor ) const;
    bool eval_erase(int at, double &savings) const;
	//for cost function
	Trashnode C , last;
	double ttSC, ttDC, ttCD, ttDE, ttCC;
	double realttSC, realttDC, realttCD, realttDE, realttCC;
	int N, Nreal,minDumpVisits,maxDumpVisits,realDumpVisits;
        int Z,z1,z2,realz1,realz2,n,z,Zmissing,lastn;
	double arrivalEclosesLast,realArrivalEclosesLast,shiftLength;
	double serviceE;
	double totalTime,realTotalTime,lastRealTotalTime;
	double forcedWaitTime,totalWaitTime,idleTime;
	double realForcedWaitTime,realtotalWaitTime,realIdleTime;
	double idleTimeSCDE,idleTimeSDCDE;
	double realIdleTimeSCDE,realIdleTimeSDCDE;
	double sumIdle,penalty;
	inline int  realN() const { return ( path.getDumpVisits() + 1 ) ;}
	inline double  totalServiceTime() { 
		return ( path.getTotServiceTime()+  dumpSite.getservicetime() + endingSite.getservicetime() ) ;}  
        double v_cost,workNotDonePerc;
        double getCost() const { return v_cost;};	
        double getCost() {setCost();  return v_cost;};	
	int getz1() const  {return realz1;};
	int getz2() const {return realz2;};
	int getn() const {return n;};

        void setInitialValues( const Trashnode &node, const Bucket &picks){

		C = node;
		ttSC=twc->getAverageTime(depot,picks);
		ttDC=twc->getAverageTime(dumpSite,picks);
		ttCD=twc->getAverageTime(picks,dumpSite);
		ttDE=twc->TravelTime(dumpSite,endingSite);
		ttCC=twc->TravelTime(C,C);
		serviceE=endingSite.getservicetime();
                shiftLength=endTime-startTime;
		e_makeFeasable(0);
		Z= floor( maxcapacity/C.getdemand());
		arrivalEclosesLast = C.closes() + ttCD + dumpSite.getservicetime() + ttDE;
		totalTime=0;
		
		N =-1; 
		do {
		N++;
		totalTime=depot.getservicetime()  + ttSC + ttDE + endingSite.getservicetime() 
			+ N * Z * C.getservicetime() + N * (Z - 1) * ttCC + (N -1) * ttDC
			+ N * ( dumpSite.getservicetime() + ttCD );
		}
		while ( totalTime < arrivalEclosesLast + serviceE );
		forcedWaitTime=endTime-( arrivalEclosesLast  +  serviceE );
                totalWaitTime=endTime- (endingSite.getArrivalTime() +serviceE);
		idleTimeSCDE= C.closes() - (depot.getservicetime() + ttSC );
		z1 = idleTimeSCDE / (C.getservicetime() + ttCC);
		idleTimeSDCDE= C.closes() - (dumpSite.getDepartureTime() + ttDC );
		z2 = idleTimeSDCDE / (C.getservicetime() + ttCC);
                idleTime= totalWaitTime-forcedWaitTime;
	};

	void setCost() {
		last=(size()>1)? path[size()-1] : C ;
		realttSC=path.size()>1? path[1].getTotTravelTime()  :ttSC;
		double deltattSC=realttSC-ttSC; // >0 viaja mas lejos para llegar al primer contenedor
	        ttSC=std::min(realttSC,ttSC);

		realttCC=size()>1?(path.getTotTravelTime()-realttSC)/(size()-1) :ttCC;
		double deltattCC=realttCC-ttCC; // >0 el promedio de viaje entre contenedores es mayor
	        ttCC=std::min(realttCC,ttCC);

		realttCD=0;
                realttDC=0;
		double realZ=0;
		if ( path.getDumpVisits() ) {
			for (int i=1;i<path.size()-1;i++) {
			    realZ++;
			    if (path[i-1].isdump()) realttCD+= twc->TravelTime( path[i-1], path[i]);
			    if (path[i].isdump()) realttDC+= twc->TravelTime( path[i], path[i+1]);
                        }
                } else realttDC=ttDC;
                realttCD= (realttCD+ twc->TravelTime(last,dumpSite)) /(path.getDumpVisits()+1.0);
		double deltattDC=realttDC-ttDC; // >0 el viaje del dump al contenedor es mas largo que lo esperado (worse)
		double deltattCD=realttCD-ttCD; // >0 el viaje del contenedor al dump es mar largo que lo esperado

	        ttCD=std::min(realttCD,ttCD);
	        ttDC=std::min(realttDC,ttDC);

		realttDE=ttDE;

		realArrivalEclosesLast = last.closes() +  last.getservicetime() + twc->TravelTime(last,dumpSite) + dumpSite.getservicetime() + realttDE;
		double deltaArrivalEclosesLast= realArrivalEclosesLast - arrivalEclosesLast; //>0 the latest the truck can arrive is better
		arrivalEclosesLast= std::max(realArrivalEclosesLast, arrivalEclosesLast); //>0 the latest the truck can arrive
		
		realForcedWaitTime=endTime-( realArrivalEclosesLast  +  serviceE );
		double deltaForcedWaitTime= realForcedWaitTime - forcedWaitTime;  //>0 bad thing the forcedWaitTime has increased
		forcedWaitTime = std::min (realForcedWaitTime , forcedWaitTime);

		n  = size() - 1 - ( realN() - 1 );  
		double deltan = n - lastn; //>0 allways good, we have one more container (truck point fo view)
		lastn = n;  //setting this n as the last

	        z = (realN()==1)?  n  : n % Z ;
		double deltaZ = Z-z;   //>0 good, we can work more containers/trip
		Z = std::max(Z,z);

		Zmissing=Z-z;   //==0 we are in the limit of container pickup
				//>0 we need to pickup more containers
				//its never negative
		assert(Zmissing>=0);

		realTotalTime= endingSite.getArrivalTime();
		double deltaRealTotalTime= realTotalTime - lastRealTotalTime;  //>0 the total time has increased  good or bad depends on deltan
		lastRealTotalTime = realTotalTime;  

		
if (realArrivalEclosesLast < realTotalTime) { last.dumpeval(); dumpCostValues();};
		assert (realArrivalEclosesLast > realTotalTime); //otherwise we are in a TWV and something is wrong on the calculation
		realIdleTime =  realArrivalEclosesLast -  realTotalTime ;
		realIdleTimeSCDE =  ( Zmissing>0 )?  (C.getservicetime() + realttCC ) * Zmissing :
                		            C.closes() - ( depot.getDepartureTime() +  realttSC) ;

		realz1 = std::min ( (int) (floor(realIdleTime / ( C.getservicetime() + realttCC ) )) ,Zmissing ) ;
		realIdleTimeSDCDE =  std::max ( (C.closes() - ( dumpSite.getDepartureTime() +  realttDC )) , 0.0 ); //cant have negative idleTime
		realz2 = floor(realIdleTimeSDCDE / ( C.getservicetime() +  realttCC ));
		sumIdle=realIdleTimeSCDE+realIdleTimeSDCDE+realIdleTime;

		//tengo z contenedores en el utimo viaje 
		//me faltan Zmissing contenedores para un viaje lleno al dump
		//pero solo puedo hacer z1 contenedores mas en ese viaje al dump;

                double deltaz1 = realz1 - z1; 
		if (deltan>=0) //aumente el numero de contenedores  deltaz1>0 es bueno, aumente contenedor y pudo puedo aumentar mas contenedores todavia (no tiene sentido)
			z1= std::max (z1-1, realz1);
		if (deltan==0) //el numero de contenedores no cambio  deltaz1>0 es bueno
			z1= std::max (z1, realz1) ;
		if (deltan<0)  //quite un contenedor, deltaz>0 me hace falta un contenedor z1 debe de haber aumentado minimo en 1
			z1= std::max (z1+1, realz1);
		

                double deltaz2 = realz2 - z2; 
		if (deltan>=0) //aumente el numero de contenedores  deltaz2>0 es bueno, aumente contenedor y pudo puedo aumentar mas contenedores todavia (no tiene sentido)
			z2= std::max (z2-1, realz2);
		if (deltan==0) //el numero de contenedores no cambio  deltaz2>0 es bueno
			z2= std::max (z2, realz2) ;
		if (deltan<0)  //quite un contenedor, deltaz>0 me hace falta un contenedor z1 debe de haber aumentado minimo en 1
			z2= std::max (z2+1, realz2);
		
#ifdef TESTED		
	std::cout<<"TODOS LOS DELTAS2"
		<<"deltattSC    "<<deltattSC    <<"\n"
		<<"deltattCC    "<<deltattCC    <<"\n"
		<<"deltattDC    "<<deltattDC    <<"\n"
		<<"deltattCD    "<<deltattCD    <<"\n"
		<<"deltaArrivalEclosesLast    "<<deltaArrivalEclosesLast    <<"\n"
		<<"deltaForcedWaitTime    "<<deltaForcedWaitTime    <<"\n"
		<<"deltan    "<<deltan    <<"\n"
		<<"deltaZ    "<<deltaZ    <<"\n"
		<<"deltaRealTotalTime    "<< deltaRealTotalTime    <<"\n"
		<<"deltaz1    "<<deltaz1    <<"\n"
		<<"deltaz2    "<<deltaz2    <<"\n";
#endif


		workNotDonePerc=(double (realz1 + realz2))  /(double (double(n) + double(realz1) +double(realz2) ));
		double workDonePerc=1-workNotDonePerc;
		
		v_cost=  /* realTotalTime * (1 + workNotDonePerc) + sumIdle * ( 1 + workDonePerc) +*/ getduration();
	};

	double getDeltaCost(double deltaTravelTime,int deltan) {
		double newrealTotalTime=realTotalTime+deltaTravelTime;
		double newrealIdleTime= realArrivalEclosesLast - newrealTotalTime;
		int newn=n+deltan;
		int newz= (realN()==1)?  newn  : newn % Z ;
		int newZmissing= (Z>newz)? Z-newz:0;
		double newrealIdleTimeSCDE =  ( newz )? newrealIdleTime - (C.getservicetime() + realttCC ) * newZmissing :
                                           C.closes() - ( depot.getDepartureTime() +  realttSC);
		double newrealz1 = std::min ( (int) (floor(newrealIdleTime / ( C.getservicetime() + realttCC ) )) ,newZmissing ) ;
                double newrealIdleTimeSDCDE =  C.closes() - ( dumpSite.getDepartureTime() + deltaTravelTime + realttDC );
                double  newrealz2 = newrealIdleTimeSDCDE / ( C.getservicetime() +  realttCC );

                double newv_cost= newrealTotalTime + (newrealz1+newrealz2)*newn + newrealIdleTimeSCDE + newrealIdleTimeSDCDE;
		double deltacost= newv_cost-v_cost;
		return deltacost;
	};


		

void dumpCostValues() const{
	std::cout<<" +++++++++++++++++++++  	 TRUCK #<<"<<vid<<"      +++++++++++++++++++++ \n\n\n"; 
	std::cout<<" Average Container \t"; 
	C.dump();
	std::cout<<" ------  current path -------\n";
	tau();
		
	std::cout<<" ------  truck time limits -------\n"
		<<"Shift Starts\t"<<startTime<<"\n"
		<<"Shift ends\t"<<endTime<<"\n"
		<<"Shift length\t"<<shiftLength<<"\n";

		
	std::cout<<"\n\n\n ------Real  Values of current truck in the solution -------\n"
                <<"                   realttSC\t"<<realttSC<<"\n"
                <<"                   realttCC\t"<<realttCC<<"\n"
                <<"                   realttCD\t"<<realttCD<<"\n"
                <<"                   realttDC\t"<<realttDC<<"\n"
                <<"                   realttDE\t"<<realttDE<<"\n"
		<<"                 service(E)\t"<<serviceE<<"\n"
		<<"                maxcapacity\t"<<maxcapacity<<"\n"
		<<"              C.getdemand()\t"<<C.getdemand()<<"\n"
		<<"         C.getservicetime()\t"<<C.getservicetime()  <<"\n"
		<<"                 C.closes()\t"<<C.closes()<<"\n"
		<<"    path[size()-1].closes()\t"<< path[size()-1].closes() <<"\n"
		<<"  dumpSite.getservicetime()\t"<<dumpSite.getservicetime()  <<"\n"
		<<"dumpSite.getDepartureTime()\t"<< dumpSite.getDepartureTime() <<"\n"
		<<"                      realN\t"<<realN()  <<"\n"
		<<"endingSite.getArrivalTime()\t"<<endingSite.getArrivalTime()  <<"\n"
		<<"   depot.getDepartureTime()\t"<<depot.getDepartureTime() <<"\n"
		<<"                       size\t"<<size() <<"\n"
		//<<" \t"<<  <<"\n"
		//<<" \t"<<  <<"\n"
		//<<" \t"<<  <<"\n"

		<<"                     Z =\t"<<Z  <<"\t= floor( maxcapacity/C.getdemand() )\t"<<Z<<"\n"
		<<"                     n =\t"<<n  <<"\t=size() - 1 - ( realN()-1 )  \t"<< n<< "\n"
	        <<"                     z =\t"<< z <<"\t=(realN()==1)  z = n  : n % Z\t"  <<  z <<"\n" 
		<<"              Zmissing =\t"<< Zmissing <<"\t=Z-z\t"<<Zmissing<<"\n"
		<<"                realz1 =\t"<< realz1 <<"\t== min ( floor ( realIdleTimeSCDE / (C.getservicetime() + realttCC) ) , Zmissing )\t"<<realz1<<"\n"
		<<"                realz2 =\t"<< realz2 <<"\t=idleTimeSDCDE / (C.getservicetime() + realttCC)\t"	<<realz2<<"\n"

                <<"realArrivalEclosesLast =\t"<< realArrivalEclosesLast <<"\t=path[size()-1].closes() + realttCD + dumpSite.getservicetime() + realttDE \t"<< realArrivalEclosesLast<< " \n "
                <<"    realForcedWaitTime =\t"<<realForcedWaitTime  <<"\t=shiftEnds -( realArrivalEclosesLast  +  serviceE )\t" <<realForcedWaitTime<<"\n"
		<<"         realTotalTime =\t"<<realTotalTime  <<"\t=endingSite.getArrivalTime()\t"<< realTotalTime<<"\n"

		
		<<"          realIdleTime =\t"<< realIdleTime <<"\t=realArrivalEclosesLast -  realTotalTime\t"<<realIdleTime<<"\n" 
		<<"      realIdleTimeSCDE =\t"<<realIdleTimeSCDE  <<"\t=( Zmissing>0 )?  (C.getservicetime() + realttCC ) * Zmissing :\n"
                                           "\t\t(Zmissing==0? C.closes() - ( depot.getDepartureTime() +  realttSC):0) ;\t"<<realIdleTimeSCDE<<"\n"
		<<"     realIdleTimeSDCDE =\t"<<realIdleTimeSDCDE  <<"\t=C.closes() - ( dumpSite.getDepartureTime() + realttDC)\t"<<realIdleTimeSDCDE<<"\n"
	
		<<"                sumIdle=\t"<< sumIdle <<"\t=sumIdle=realIdleTimeSCDE+realIdleTimeSDCDE+realIdleTime\t"<<sumIdle<<"\n"

		<<"        workNotDonePerc=\t"<<workNotDonePerc<<"\t=(double (realz1 + realz2))  /(double (n + realz1+realz2))\n"
		<<"     1+ workNotDonePerc=\t"<<(1+workNotDonePerc)<<"\t=(double (realz1 + realz2))  /(double (n + realz1+realz2))\n"
		<<"realTotalTime + sumIdle)=\t"<<(realTotalTime + sumIdle)<<"\n"
		<<"\n\n             v_cost=\t"<<v_cost<<"\t= (realTotalTime + sumIdle) *( 1 + workNotDonePerc)\n";
		std::cout<<"\n";

/*
		<<"\n\n\n DELTA TIME SIMULATION\n"
		<<"if a container is added into a very full truck:\n";

		for (double delta=-20; delta<20;delta++) { //changes in time  
			if (n) {
				std::cout<<"same amount of containers delta="<<delta<<  "\t    delta+delta/n=" <<(penalty=delta/n)<<"\t";
				std::cout<<"penalty*sumIdle= "<<(penalty*sumIdle)<<"\n";
			}
			if (n+1){
				 std::cout<<"1 container more          delta="<<delta<<"\tdelta+delta/(n+1)="<<(delta/(n+1))<<"\t";
				std::cout<<"penalty*sumIdle= "<<(penalty*sumIdle)<<"\n";
			}
			if (n-1) {
				 std::cout<<"1 container less          delta ="<<delta<<"\tdelta+delta/(n-1)="<<(delta/(n-1))<<"\t";
				std::cout<<"penalty*sumIdle= "<<(penalty*sumIdle)<<"\n";
			}
		}
*/
;

/*
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
*/
	};

};


#endif

