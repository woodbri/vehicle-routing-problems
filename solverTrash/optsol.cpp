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

#include "timer.h"
#include "trashstats.h"
#include "optsol.h"


/**
- checks if a truck can be reduced

- in case it can be reduced
	tries the reduction

TODO  refine the code
*/
void OptSol::optimizeTruckNumber()   {
	/** Stores the trucks position in the fleet */
	std::deque<int> fullz1;    /**< Trucks that cant receive a container in the current trip  */
	std::deque<int> fullz2;     /**< Trucks that cant receive a container in the next (non-existing) trip */
	std::deque<int> notFullz1; /**< Trucks that CAN receive a container in the current trip  */
	std::deque<int> notFullz2; /**< Trucks that CAN  receive a container in the next (non-existing) trip  */
	std::deque<int> allTrucks; /**< All trucks  */

	int z1Tot=0;		   /**< total number of containers that can be picked in the current trip */
	int z2Tot=0;		   /**< total number of containers that can be picked in the next (non-exisiting)  trip */
	int minn=datanodes.size(); /**< setting a minimum to see if it is requiered to minimize */
	int truckWithMinn=-1;      /**< the trucks position that has that minimun */
	int z1AtMin=0;      	   /**< the trucks that has the min number of containers has the most number of avaliable spots in the current trip*/
	int z2AtMin=0;      	   /**< the trucks that has the min number of containers has the most number of avaliable spots in the next (non-exisiting) trip*/

	/** requiered by the evaluation of a move */
	int fromTruck; 		  /**< truck from where a container is moved */
	bool emptiedTruck=false;

	for (int i=0;i<fleet.size();i++) {
		allTrucks.push_back(i);

		if (fleet[i].getz1()) {
			notFullz1.push_back(i);
			z1Tot+=fleet[i].getz1();
		} else fullz1.push_back(i);

		if (fleet[i].getz2()) {
			notFullz2.push_back(i);
			z2Tot+=fleet[i].getz2();
		} else fullz2.push_back(i);

		if ( fleet[i].getn()<minn) {
			minn= fleet[i].getn();
			truckWithMinn=i;
			z1AtMin=fleet[i].getz1();
			z2AtMin=fleet[i].getz2();
		}
	}
	fromTruck=truckWithMinn;

#ifndef LOG
std::cout<<"fromTruck"<<fromTruck<<"\n";
std::cout<<"need to fit "<<minn<<"containers into \t"<<(z1Tot-z1AtMin);
std::cout<<"= z1Tot "<<z1Tot<<" - ";
std::cout<<" z1AtMin "<<z1AtMin<<"\n OR \n";

std::cout<<"need to fit "<<minn<<"containers into \t"<<(z2Tot-z2AtMin);
std::cout<<"= z2Tot "<<z2Tot<<" - ";
std::cout<<" z2AtMin "<<z2AtMin<<"\n";

std::cout<<"not FUll Trucks in current Trip\n";
for (int i=0;i<notFullz1.size();i++) 
   std::cout<<notFullz1[i]<<"\t";
std::cout<<"\n";
std::cout<<" FUll Trucks in current trip\n";
for (int i=0;i<fullz1.size();i++) 
   std::cout<<fullz1[i]<<"\t";
std::cout<<"\n";

std::cout<<"not FUll Trucks in next (non-existing) Trip\n";
for (int i=0;i<notFullz2.size();i++) 
   std::cout<<notFullz2[i]<<"\t";
std::cout<<"\n";
std::cout<<" FUll Trucks in next (non-exisiting) trip\n";
for (int i=0;i<fullz2.size();i++) 
   std::cout<<fullz2[i]<<"\t";
std::cout<<"\n";
#endif

tau();
	if (minn<= (z1Tot-z1AtMin) ) { //a truck can be removed without extra trip in any other truck

            emptiedTruck=emptyTheTruck(fromTruck,notFullz1);  //first try to remove the smallest truck
            if (not emptiedTruck) 
	   	emptiedTruck=emptyAtruck(notFullz1,notFullz1);
	    	
 	} 
	if (not emptiedTruck and minn<= (z2Tot-z2AtMin) ) {
	    fromTruck=truckWithMinn;
            emptiedTruck=emptyTheTruck(fromTruck,notFullz2);
            if (not emptiedTruck) 
		emptiedTruck = emptyAtruck(notFullz2,notFullz2);
	}
	if (not emptiedTruck and ( minn<= (z2Tot-z2AtMin) or minn<= (z1Tot-z1AtMin) ) ) {
		emptiedTruck = emptyAtruck(allTrucks,allTrucks);
	}
#ifndef LOG
std::cout<<"\n"; tau();
#endif
}

bool OptSol::emptyAtruck(std::deque<int> fromThis,std::deque<int> intoThis) {
       int fromTruck; 				/**< truck & position from where a container is moved */
       for (int i=0;i<fromThis.size();i++) {
            fromTruck=fromThis[i];
            if (emptyTheTruck(fromTruck,intoThis)) {
                   return true;
            }
       }
}
			

bool OptSol::emptyTheTruck(int fromTruck, std::deque<int> notFull) {
	std::deque<Move> moves;    	/**< moves storage */
	int fromPos=1;			/**< postition of a container in the fromTruck */
	int toTruck; 			/**< truck to  where a container is moved */
	double savings;			/**< savings of the move */
	double factor=1;		/**< factor=1 making sure all feasable moves are returned */
        int count=fleet[fromTruck].getn(); /**< number of containers in truck */
        for (int j=0;j<count;j++) {
                moves.clear();

                for (int i=0;i<notFull.size();i++) {
                   toTruck=notFull[i];
                   if (toTruck==fromTruck) continue;
		   if ( not fleet[fromTruck].eval_erase(fromPos,savings,twc) ) continue; //for whatever reason erasing a node makes the truck infeasable 
                   fleet[toTruck].eval_insertMoveDumps( fleet[fromTruck][fromPos], moves, fromTruck, fromPos, toTruck, savings, factor, twc );
                };

                if (moves.size()) {
                  std::sort(moves.begin(), moves.end(), Move::bySavings);
                  v_applyMove(moves[0]);
                } else {
                        fromPos++;
                }
                if (fromPos>=fleet[fromTruck].size()) break;
            }
        if (fleet[fromTruck].size()==1) {
                fleet.erase(fleet.begin()+fromTruck);
		return true;
        } else return false;
}



	


void OptSol::v_getIntraSwNeighborhood(std::deque<Move>& moves, double factor)  const {
    moves.clear();

    // iterate through each vehicle (vi)
    int truckPos=intraTruckPos;

std::cout<<"working with truck "<<truckPos<<" intraSw neighborhood\n";
    fleet[truckPos].eval_intraSwapMoveDumps( moves,  truckPos, factor, twc); 
    moves[0].dump();    
    if ( (moves.size()==0) or (moves[0].getsavings()<0))  
       if  (intraTruckPos==fleet.size()-1 ) intraTruckPos=0;
       else intraTruckPos++;
}


void OptSol::v_getInterSwNeighborhood(std::deque<Move>& moves, double factor)  const {
    assert (feasable());
    if (not fleet.size())  return;    

    int truckPos=interTruckPos1;
    int otherTruckPos=interTruckPos2;
    
    if  (interTruckPos1==fleet.size()-2 and interTruckPos2==fleet.size()-1) {interTruckPos1=0; interTruckPos2=1;}
    else if (interTruckPos1<fleet.size()-2 and interTruckPos2==fleet.size()-1) { interTruckPos1++; interTruckPos2=interTruckPos1+1;}
    else if (interTruckPos2<fleet.size()-1) interTruckPos2++;


    moves.clear();
std::cout<<"working with truck "<<truckPos<<" and"<< otherTruckPos<<"interSw neighborhood\n";

    // iterate through the vechicles (vi, vj)
//    for (int truckPos=0; truckPos < fleet.size(); truckPos++) {  
//        for (int otherTruckPos =truckPos + 1; otherTruckPos < fleet.size(); otherTruckPos++) { //testNeeded
//            assert (not (truckPos == otherTruckPos) ); 

            for (int fromPos=1; fromPos<fleet[truckPos].size(); fromPos++) {
		if(fleet[truckPos][fromPos].isdump()) continue;   // skiping dump
		
		fleet[truckPos].eval_interSwapMoveDumps( moves, fleet[otherTruckPos], truckPos, otherTruckPos, fromPos, factor); 
            }
//        }
//    }
}



void OptSol::v_getInsNeighborhood(std::deque<Move>& moves, double factor) const {
     v_getInsNeighborhood( moves, factor,0);
};


void OptSol::v_getInsNeighborhood(std::deque<Move>& moves, double factor, int count) const  {

#ifdef TESTED
std::cout<<"Entering OptSol::v_getInsNeighborhood for "<<fleet.size()<<" trucks \n";
#endif
assert (feasable());

    moves.clear();
    double savings;
    if ((insTruckPos1>= fleet.size()) or (insTruckPos2>= fleet.size())) {
           interTruckPos1=insTruckPos1=fleet.size()-1;
           interTruckPos2=insTruckPos2=0;
    };
    if (insTruckPos1 == insTruckPos2) return;

    int fromTruck=insTruckPos1;
    int toTruck=insTruckPos2;


#ifndef TESTED
std::cout<<"\n\n\n\n**********************************working with truck "<<fromTruck<<" and "<< toTruck<<" insSw neighborhood\n";
#endif
    if (fleet[toTruck].getz1() or fleet[toTruck].getz2()) { //only try if there is a possibility to insert a container

          for (int fromPos=1; fromPos<fleet[fromTruck].size(); fromPos++) {
		if(fleet[fromTruck][fromPos].isdump()) continue;   // skiping dump
        	if (fleet[ fromTruck ].size()==1) {
			std::cout<<" A TRUCK WITHOUT CONTAINERS HAS BEING GENERATED";
        		//trucks.push_back(fleet[ fromTruck   ]);
        		//fleet.erase(fleet.begin() + fromTruck ); 
			interTruckPos1=insTruckPos1=fleet.size()-2;
			interTruckPos2=insTruckPos2=0;
			return;
		};
		if ( not fleet[fromTruck].eval_erase(fromPos,savings,twc) ) continue; //for whatever reason erasing a node makes the truck infeasable 
		//fleet[fromTruck].eval_erase(fromPos,savings,twc);
                fleet[toTruck].eval_insertMoveDumps( fleet[fromTruck][fromPos], moves, fromTruck, fromPos, toTruck, savings, factor ,twc);
          }
    }
    	insTruckPos2++;
    	if (insTruckPos1 == insTruckPos2) insTruckPos2++;
    	if (insTruckPos2 == fleet.size()) {insTruckPos1++; insTruckPos2=0;};
    	if (insTruckPos1 == fleet.size()) {insTruckPos1=0; insTruckPos2=1;};
	assert(insTruckPos1 != insTruckPos2);

#ifdef TESTED
std::cout<<"EXIT OptSol::v_getInsNeighborhood "<<moves.size()<<" MOVES found total \n";
#endif
}
 
bool OptSol::v_applyInsMove( const Move &move) {
	assert(move.getmtype() == Move::Ins);
	assert( fleet[ move.getInsFromTruck() ].feasable() );
	assert( fleet[ move.getInsToTruck() ].feasable() );
	fleet[ move.getInsFromTruck() ].getCost(twc);
	fleet[ move.getInsToTruck() ].getCost(twc);
#ifdef TESTED
move.dump();
fleet[ move.getInsFromTruck() ].dumpCostValues();
fleet[ move.getInsToTruck() ].dumpCostValues();
#endif
	fleet[ move.getInsFromTruck() ].applyMoveINSerasePart(move.getnid1(), move.getpos1());
        fleet[ move.getInsToTruck() ].applyMoveINSinsertPart(datanodes[ move.getnid1() ], move.getpos2());

	fleet[ move.getInsFromTruck() ].getCost(twc);
	fleet[ move.getInsToTruck() ].getCost(twc);

#ifdef TESTED
fleet[ move.getInsFromTruck() ].dumpCostValues();
fleet[ move.getInsToTruck() ].dumpCostValues();
assert(true==false);
#endif


	assert( fleet[ move.getInsFromTruck() ].feasable() );
	assert( fleet[ move.getInsToTruck() ].feasable() );



	return (fleet[ move.getInsFromTruck() ].feasable() and  fleet[ move.getInsToTruck() ].feasable() );
}



void OptSol::v_applyMove(const Move& m)  {

    switch (m.getmtype()) {
        case Move::Ins:
            {
                applyInsMove( m );
                assert( fleet[m.getvid1()].feasable() ); //just in case
                assert( fleet[m.getvid2()].feasable() );
            }
            break;
        case Move::IntraSw:
            {
                applyIntraSwMove( m );
                assert( fleet[m.getvid1()].feasable() );
            }
            break;
        case Move::InterSw:
            {
                applyInterSwMove( m );
                assert( fleet[m.getvid1()].feasable() );
                assert( fleet[m.getvid2()].feasable() );
            }
            break;
    }
}



