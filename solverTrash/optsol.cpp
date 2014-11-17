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
#include "stats.h"
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

#ifdef LOG
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
#ifdef LOG
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
	Moves moves;    	/**< moves storage */
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
		   if ( not fleet[fromTruck].eval_erase(fromPos,savings) ) continue; //for whatever reason erasing a node makes the truck infeasable 
                   fleet[toTruck].eval_insertMoveDumps( fleet[fromTruck][fromPos], moves, fromTruck, fromPos, toTruck, savings, factor );
                };

                if (moves.size()) {
                  v_applyMove( *(moves.begin()) );
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



	

/*
void OptSol::v_getIntraSwNeighborhood(Moves &moves, double factor)  const {
std::cout<<"THIS SHOULDNT BE CALLED ";
    moves.clear();

    // iterate through each vehicle (vi)
    int truckPos=intraTruckPos;

std::cout<<"working with truck "<<truckPos<<" intraSw neighborhood\n";
    fleet[truckPos].eval_intraSwapMoveDumps( moves,  truckPos, factor, twc); 
    moves.begin()->Dump();    
    if ( (moves.size()==0) or (moves.begin()->getsavings()<0))  
       if  (intraTruckPos==fleet.size()-1 ) intraTruckPos=0;
       else intraTruckPos++;
}
*/
void OptSol::getIntraSwNeighborhood(Move::Mtype mtype,  Moves &moves, double factor)  const {
    moves.clear();
    if (fleet.size()==1)  {
       fleet[0].eval_intraSwapMoveDumps( moves,  0, factor);
       return; 
    }
    switch (mtype) {
	case Move::InterSw:
std::cout<<"IntraSw working with truck "<<interTruckPos1<<" intraSw neighborhood\n";
	    fleet[interTruckPos1].eval_intraSwapMoveDumps( moves,  interTruckPos1, factor);
std::cout<<"IntraSw working with truck "<<interTruckPos2<<" intraSw neighborhood\n";
	    fleet[interTruckPos2].eval_intraSwapMoveDumps( moves,  interTruckPos2, factor);
	    moves.begin()->Dump();
	    break;
	case Move::Ins:
std::cout<<"IntraSw working with truck "<<insTruckPos1<<" intraSw neighborhood\n";
	    fleet[insTruckPos1].eval_intraSwapMoveDumps( moves,  insTruckPos1, factor);
std::cout<<"IntraSw working with truck "<<insTruckPos2<<" intraSw neighborhood\n";
	    fleet[insTruckPos2].eval_intraSwapMoveDumps( moves,  insTruckPos2, factor);
	    moves.begin()->Dump();
	    break;
    }
}






void OptSol::getInterSwNeighborhood(Moves &moves, double factor)  const {
    assert (feasable());
    if (not fleet.size())  return;   //no trucks in solution 
    if (fleet.size()==1)  return;   //no intersw in 1 truck solution 

    if (factor < 0.996 and fleet.size()>2) { //factor greater than 0.996 means to stay in the same neighborhood as last time
    	if  (interTruckPos1==fleet.size()-2 and interTruckPos2==fleet.size()-1) {interTruckPos1=0; interTruckPos2=1;}
    	else if (interTruckPos1<fleet.size()-2 and interTruckPos2==fleet.size()-1) { interTruckPos1++; interTruckPos2=interTruckPos1+1;}
    	else if (interTruckPos2<fleet.size()-1) interTruckPos2++;
	if (interTruckPos1==interTruckPos2) {interTruckPos1=0; interTruckPos2=1;}
    }

    int truckPos=interTruckPos1;
    int otherTruckPos=interTruckPos2;
    
    moves.clear();
#ifndef LOG
std::cout<<"InterSw working with truck "<<truckPos<<" and"<< otherTruckPos<<"interSw neighborhood\n";
#endif
    fleet[truckPos].eval_interSwapMoveDumps( moves, fleet[otherTruckPos], truckPos, otherTruckPos, factor); 


#ifdef CHECK
    Move move;
    bool flag=true;
    for(MovesItr movePtr=moves.begin(); movePtr!=moves.end();++movePtr) {
           move = (*movePtr);
           if ( not testInterSwMove(move) ) {
			move.Dump();
			flag=false;
           }
    }
    assert(flag);
#endif

}



void OptSol::getInsNeighborhood(Moves &moves, double factor) const {

#ifdef TESTED
std::cout<<"Entering OptSol::v_getInsNeighborhood for "<<fleet.size()<<" trucks \n";
#endif
assert (feasable());
    if (fleet.size()==1)  return;   //no ins in 1 truck solution 
    moves.clear();
    double savings;
    if ((insTruckPos1>= fleet.size()) or (insTruckPos2>= fleet.size())) {
           insTruckPos1=insTruckPos1=fleet.size()-1;
           insTruckPos2=insTruckPos2=0;
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
		if ( not fleet[fromTruck].eval_erase(fromPos,savings) ) continue; //for whatever reason erasing a node makes the truck infeasable 
		//fleet[fromTruck].eval_erase(fromPos,savings,twc);
                fleet[toTruck].eval_insertMoveDumps( fleet[fromTruck][fromPos], moves, fromTruck, fromPos, toTruck, savings, factor );
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

// 2 vehicles involved
 bool OptSol::v_applyInterSwMove( const Move &move) {
  assert(move.getmtype() == Move::InterSw);
  assert(not (move.getInterSwTruck1()==move.getInterSwTruck2()));

  if (not (fleet[move.getInterSwTruck1()][ move.getpos1()].getnid() == move.getnid1() )) {
        move.Dump();
	fleet[move.getInterSwTruck1()][ move.getpos1()].dump();
	fleet[move.getInterSwTruck2()][ move.getpos2()].dump();
	std::cout<<"\n";
  }
	
  assert(fleet[move.getInterSwTruck1()][ move.getpos1()].getnid() == move.getnid1() );
  assert(fleet[move.getInterSwTruck2()][ move.getpos2()].getnid() == move.getnid2() );
 
  fleet[move.getInterSwTruck1()].applyMoveInterSw( fleet[move.getInterSwTruck2()], move.getpos1(),move.getpos2() ) ;
 
  assert( fleet[ move.getInterSwTruck1() ].feasable() );
  assert( fleet[ move.getInterSwTruck2() ].feasable() );
  return (fleet[ move.getInterSwTruck1() ].feasable() and fleet[ move.getInterSwTruck2() ].feasable() );
 }

 bool OptSol::testInterSwMove( const Move &move) const {
//move.Dump();
  if ( not (move.getmtype() == Move::InterSw) ) return false;
  if ( (move.getInterSwTruck1()==move.getInterSwTruck2())) return false;
  if ( not ( move.getInterSwTruck1() < fleet.size()) ) return false;
  if ( not ( move.getInterSwTruck2() < fleet.size()) ) return false;
  if ( not ( move.getpos1() < fleet[move.getInterSwTruck1()].size()) ) return false;
  if ( not ( move.getpos2() < fleet[move.getInterSwTruck2()].size()) ) return false;
  if ( not (fleet[move.getInterSwTruck1()][ move.getpos1()].getnid() == move.getnid1() )) return false;
  if ( not (fleet[move.getInterSwTruck2()][ move.getpos2()].getnid() == move.getnid2() ))return false;

  Vehicle truck=fleet[move.getInterSwTruck1()];
  Vehicle other=fleet[move.getInterSwTruck2()];

  return truck.applyMoveInterSw( other, move.getpos1(),move.getpos2() )  ;
  
 }


bool OptSol::testInsMove( const Move &move) const {
//move.Dump();
  if (not move.getmtype() == Move::Ins) return false;
  if ( (move.getInsFromTruck()==move.getInsToTruck()) ) return false;
  if ( move.getInsFromPos() >= fleet[move.getInsFromTruck()].size() ) return false;
  if ( not move.getInsFromTruck() < fleet.size() ) return false;
  if ( not (fleet[move.getInsFromTruck()] [ move.getInsFromPos()].getnid() == move.getInsNid() )) return false;

  Vehicle truck=fleet[move.getInsFromTruck()];
  Vehicle other=fleet[move.getInsToTruck()];
  truck.applyMoveINSerasePart(move.getnid1(), move.getpos1());
  other.applyMoveINSinsertPart(datanodes[ move.getnid1() ], move.getpos2());

  return truck.feasable() and other.feasable();

}



void OptSol::v_applyMove(const Move &move)  {

    switch (move.getmtype()) {
        case Move::Ins:
            {
                applyInsMove( move );
                assert( fleet[move.getvid1()].feasable() ); //just in case
                assert( fleet[move.getvid2()].feasable() );
            }
            break;
        case Move::IntraSw:
            {
                applyIntraSwMove( move );
                assert( fleet[move.getvid1()].feasable() );
            }
            break;
        case Move::InterSw:
            {
//assert (testInterSwMove(move)) ;
                applyInterSwMove( move );
                assert( fleet[move.getvid1()].feasable() );
                assert( fleet[move.getvid2()].feasable() );
            }
            break;
    }
}


