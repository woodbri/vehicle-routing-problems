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



void OptSol::optimizeTruckNumber()   {
	std::deque<int> fullz1,fullz2;
	std::deque<int> notFullz1,notFullz2,allTrucks;
	std::deque<Move> moves;
	int z1Tot=0;
	int z2Tot=0;
	int minn=datanodes.size();
	int truckWithMinn=-1;
	int fromTruck,toTruck,fromPos;
	double savings;
	double factor=1;
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
		}
	}
	fromTruck=truckWithMinn;
std::cout<<"fromTruck"<<fromTruck<<"\n";
std::cout<<"not FUll Trucks \n";
for (int i=0;i<notFullz1.size();i++) 
   std::cout<<notFullz1[i]<<"\t";
std::cout<<"\n";
std::cout<<" FUll Trucks \n";
for (int i=0;i<fullz1.size();i++) 
   std::cout<<fullz1[i]<<"\t";
std::cout<<"\n";


 
	if (minn<=z1Tot) { //a truck can be removed without extra trip in any other truck
            //for (int i=0;i<notFullz1.size();i++) {
	    	//fromTruck=notFullz1[i];
            for (int i=0;i<allTrucks.size();i++) {
	    	fromTruck=allTrucks[i];
	    	if (emptyTheTruck(fromTruck,allTrucks)) {
			std::cout<<"Truck is empty now\n";
			fleet.erase(fleet.begin()+fromTruck);
dumpCostValues();
			break;
	    	}
            }
 	}

assert(true==false);
}

			

bool OptSol::emptyTheTruck(int fromTruck, std::deque<int> notFull) {
	std::deque<Move> moves;
	int fromPos=1;
	int toTruck;
        double savings, factor;
            int count=fleet[fromTruck].getn();
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
moves[0].Dump();
                  v_applyMove(moves[0]);
                } else {
std::cout<<"no feasable move found for "<<fromPos<<"\n";
                        fromPos++;
                }
                if (fromPos>=fleet[fromTruck].size()) break;
            }
        return (fleet[fromTruck].size()==1);
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
    // iterate through the vechicles (vi, vj)
//    for (int fromTruck=0; fromTruck<fleet.size(); fromTruck++) {
//        for (int toTruck=0; toTruck<fleet.size(); toTruck++) {

  //          if (fromTruck==toTruck) continue;
          for (int fromPos=1; fromPos<fleet[fromTruck].size(); fromPos++) {
		if(fleet[fromTruck][fromPos].isdump()) continue;   // skiping dump
        	if (fleet[ fromTruck ].size()==1) {
			std::cout<<" A TRUCK WITHOUT CONTAINERS HAS BEING GENERATED";
        		//trucks.push_back(fleet[ fromPos   ]);
        		//fleet.erase(fleet.begin() + fromPos ); 
			interTruckPos1=insTruckPos1=fleet.size()-2;
			interTruckPos2=insTruckPos2=0;
			return;
		};
		if ( not fleet[fromTruck].eval_erase(fromPos,savings,twc) ) continue; //for whatever reason erasing a node makes the truck infeasable 
		//fleet[fromTruck].eval_erase(fromPos,savings,twc);
                fleet[toTruck].eval_insertMoveDumps( fleet[fromTruck][fromPos], moves, fromTruck, fromPos, toTruck, savings, factor ,twc);
          }
    //if (not moves.size())  {
    	insTruckPos2++;
    	if (insTruckPos1 == insTruckPos2) insTruckPos2++;
    	if (insTruckPos2 == fleet.size()) {insTruckPos1++; insTruckPos2=0;};
    	if (insTruckPos1 == fleet.size()) {insTruckPos1=0; insTruckPos2=1;};
	assert(insTruckPos1 != insTruckPos2);
    //}
//        if (count>4) return;
//	for (int i=0;i<moves.size();i++) 
//		if (moves[i].getsavings()> (-count/20)) return;
//        v_getInsNeighborhood( moves, factor+0.1,count+1);
       // }
    //}
#ifdef TESTED
std::cout<<"EXIT OptSol::v_getInsNeighborhood "<<moves.size()<<" MOVES found total \n";
#endif
}
 
bool OptSol::v_applyInsMove( const Move &move) {
	assert(move.getmtype() == Move::Ins);
	assert( fleet[ move.getInsFromTruck() ].feasable() );
	assert( fleet[ move.getInsToTruck() ].feasable() );
	fleet[ move.getInsFromTruck() ].getcost(twc);
	fleet[ move.getInsToTruck() ].getcost(twc);
#ifdef TESTED
move.dump();
fleet[ move.getInsFromTruck() ].dumpCostValues();
fleet[ move.getInsToTruck() ].dumpCostValues();
#endif
	fleet[ move.getInsFromTruck() ].applyMoveINSerasePart(move.getnid1(), move.getpos1());
        fleet[ move.getInsToTruck() ].applyMoveINSinsertPart(datanodes[ move.getnid1() ], move.getpos2());

	fleet[ move.getInsFromTruck() ].getcost(twc);
	fleet[ move.getInsToTruck() ].getcost(twc);

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
                v_applyInsMove( m );
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



