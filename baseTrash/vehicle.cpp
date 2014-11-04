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


#include <iostream>
#include <sstream>
#include <deque>

#include "trashstats.h"
#include "timer.h"

#include "trashconfig.h"
#include "twpath.h"
#include "osrm.h"
#include "move.h"
#include "vehicle.h"
#include "basevehicle.h"


// from the second truck point of view
long int Vehicle::eval_intraSwapMoveDumps( std::deque<Move> &moves, int  truckPos, int fromPo,  double factor) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_interSwapMoveDumps \n";
#endif

        double originalCost= cost ;
        double newCost;

        Trashnode node; 
        Vehicle truck ;
        std::deque<int> unTestedPos;
        std::deque<int> impossiblePos;
        int currentPos,testingPos,fromPos;

     for (fromPos=1;fromPos<size();fromPos++) {
        if ( path[fromPos].isdump() ) continue;
        node = path[fromPos]; //saved for roll back
        truck = (*this);

        for ( int i=fromPos+1; i<size(); i++)
                if ( not path[i].isdump() ) unTestedPos.push_back(i); //cant swap with a dump

        while (unTestedPos.size()) {
             currentPos=unTestedPos.back();
             unTestedPos.pop_back();

             truck.path[fromPos]=truck.path[currentPos]; //swaping
             truck.path[currentPos]=node;

             if ( not truck.e_makeFeasable(currentPos) ) {
                impossiblePos.push_back(currentPos);
                if ( path.size()*factor > impossiblePos.size() ) return moves.size();
             } else {
                assert ( truck.feasable() );
                newCost = truck.cost ; //deltaCost= newCost - originalCost

                truck.path[currentPos] = truck.path[fromPos];
                //truck.path[frompPos] will get another value so no need to roll back

                Move move(Move::IntraSw, node.getnid(), path[currentPos].getnid(), truckPos, truckPos, fromPos, currentPos, (originalCost-newCost)   );
                moves.push_back(move);
#ifdef TESTED
move.dump();
std::cout<<"\ttruck.cost"<<truck.cost<<"\totherTruck.cost"<< otherTruck.cost;
std::cout<<"\n";
#endif

             }
        }
     }
     return moves.size();
}

/**
   prev curr next
   ttpc + serv(c) + ttcn
   inf when TWV
*/
double Vehicle::timePCN(POS prev, POS curr, POS next) const  {
	if ( next==size() ) return path.timePCN(prev,curr,dumpSite);
	else return path.timePCN(prev,curr,next);
}

/**

For a truck with n containers, \f$ testedMoves= n * (n +1) / 2\f$ 

if positive savings moves are found, those are added to moves 
otherwise all the negative savings moves are added to moves

if it happens that all moves generate TWC, in that case moves does not change
if \f$ n = 0 \f$ then moves does not change

return the number of moves added to moves
*/



long int Vehicle::eval_intraSwapMoveDumps( Moves &moves, int  truckPos,  double factor, const TWC<Trashnode> &twc ) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_intraSwapMoveDumps \n";
#endif
	if (path.size()==1) return 0;
	int fromPos,withPos;
        double newCost;
	double savings;
        double deltaTime;
        //double newCargoAtNearestFromPosDump,newCargoAtCurrentNearestDump ; 
        //int moveDumpsFrom;
        //int currentNearestDumpPos;
        Vehicle truck = (*this);
	std::deque<Move>  negSavingsMoves;

        double originalCost= truck.getCost(twc);

	int originalMovesSize=moves.size();
	int deltaMovesSize=0;

//        std::deque<int> unTestedPos;
//        std::deque<int> impossiblePos;
//        std::deque<int> dumpsPos;
//        int currentPos;
//        Trashnode fromPosNearestDump;
//        bool foundFromPosNearestDump=false;
//        bool foundCurrentNearestDump;

    for (fromPos=1;fromPos<path.size()-1; fromPos++) {
	if (isdump(fromPos)) continue; //skiping dump
        Trashnode node = path[fromPos]; //saved for roll back
	for(withPos=fromPos+1; withPos<path.size();withPos++ ){
	   if (isdump(withPos)) continue; //skiping dump
/*
	   if (withPos == fromPos+ 1) {

		if (withPos+1==size()) //el que sigue es el dump
		   deltaTime=  timePCN(fromPos-1,withPos,fromPos) + timePCN(withPos,fromPos,withPos+1) - path.travelTime( path[withPos] , dumpSite )
			- (dumpSite.getArrivalTime()-path[fromPos-1].getDepartureTime());
		else
		   deltaTime=  timePCN(fromPos-1,withPos,fromPos) + timePCN(withPos,fromPos,withPos+1) - path.travelTime( path[withPos] , path[withPos+1] )
			- (path[withPos+1].getArrivalTime()-path[fromPos-1].getDepartureTime());
	   else 
		deltaTime=timePCN(fromPos-1,withPos,fromPos+1) + timePCN(withPos-1,fromPos,withPos+1)	
	  	    - ( timePCN(fromPos-1,fromPos,fromPos+1) + timePCN(withPos-1,withPos,withPos+1) );

	  //basic checking for time violation
	  if (dumpSite.deltaGeneratesTWV(deltaTime) 
		or endingSite.deltaGeneratesTWV(deltaTime)
		or path[size()-1].deltaGeneratesTWV(deltaTime) ) continue;  //Time Violation, not considered
*/
          if ( truck.applyMoveIntraSw(fromPos,  withPos) ) { //move can be done
		newCost=truck.getCost(twc);
		savings= originalCost - newCost;
		truck.applyMoveIntraSw(fromPos,  withPos) ; //roll back
                Move move(Move::IntraSw, node.getnid(), path[withPos].getnid(), truckPos, truckPos, fromPos, withPos, savings   );
		if (savings>0) {
                  moves.insert(move);
//move.dump();
		  deltaMovesSize++;
//		  if (deltaMovesSize > size()*factor) return deltaMovesSize;
		} else negSavingsMoves.push_back(move);
	  } else truck.applyMoveIntraSw(fromPos,  withPos) ; //roll back
	}
    }
    if ( deltaMovesSize ) return deltaMovesSize;
    return 0;
}


// from the second truck point of view
long int Vehicle::eval_interSwapMoveDumps( Moves &moves, const Vehicle &otherTruck,int  truckPos,int  otherTruckPos, int fromPos,  double factor,const TWC<Trashnode> &twc ) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_interSwapMoveDumps \n";
#endif

        if ( path[fromPos].isdump() ) return moves.size();

        Trashnode node = path[fromPos]; //saved for roll back
        Vehicle truck = (*this);
        Vehicle other = otherTruck;
        double originalCost= truck.getCost(twc)  + other.getCost(twc);
        double newCost,savings;
	int deltaMovesSize=0;

//        std::deque<int> unTestedPos;
//        std::deque<int> impossiblePos;
//        int currentPos,testingPos;

        for ( int i=1; i<truck.size(); i++) {
	   if (truck.path[i].isdump() ) continue;
           for ( int j=1; j<other.size(); j++) {
		if (other.path[j].isdump()) continue;
		if ( truck.applyMoveInterSw(other, i, j)) {
		   newCost=truck.getCost(twc) + other.getCost(twc);
		   savings= originalCost - newCost;
                   Move move(Move::InterSw , node.getnid(), otherTruck.path[j].getnid() ,  truckPos , otherTruckPos ,  i, j, (originalCost-newCost)   );
                   if (savings>0) {
                     moves.insert(move);
                     deltaMovesSize++;
                   } 
		}
		truck.applyMoveInterSw(other, j, i);
            }
        }
	if ( deltaMovesSize ) return deltaMovesSize;
        return 0;
}






// from the second truck point of view
long int Vehicle::eval_interSwapMoveDumps(std::deque<Move> &moves, const Vehicle &other,int  truckPos,int  otherTruckPos, int fromPos,  double factor) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_interSwapMoveDumps \n";
#endif

	if ( path[fromPos].isdump() ) return moves.size();
	double originalCost= cost + other.cost;
	double newCost;

	Trashnode node = path[fromPos]; //saved for roll back
        Vehicle truck = (*this);
        Vehicle otherTruck = other;
        std::deque<int> unTestedPos;
        std::deque<int> impossiblePos;
        int currentPos,testingPos;

        for ( int i=1; i<otherTruck.size(); i++) 
		if ( not otherTruck[i].isdump() ) unTestedPos.push_back(i); //cant swap with a dump

        while (unTestedPos.size()) {
             currentPos=unTestedPos.back();
             unTestedPos.pop_back();

             truck.path[fromPos]=otherTruck.path[currentPos]; //swaping
	     otherTruck.path[currentPos]=node;

             if ( not otherTruck.e_makeFeasable(currentPos) or not truck.e_makeFeasable(fromPos) ) {
                impossiblePos.push_back(currentPos);
                if ( otherTruck.path.size()*factor > impossiblePos.size() ) return moves.size();
             } else {
                assert ( truck.feasable() );
		assert ( otherTruck.feasable() );
		newCost = truck.cost + otherTruck.cost; //deltaCost= newCost - originalCost

	        otherTruck.path[currentPos] = truck.path[fromPos];
                //truck.path[frompPos] will get another value so no need to roll back

                Move move(Move::InterSw , node.getnid(), otherTruck.path[currentPos].getnid() ,  truckPos , otherTruckPos ,  fromPos, currentPos, (originalCost-newCost)   );
                moves.push_back(move);

#ifdef TESTED
move.dump();
std::cout<<"cost"<<cost<<"\tother.cost"<< other.cost;
std::cout<<"\ttruck.cost"<<truck.cost<<"\totherTruck.cost"<< otherTruck.cost;
std::cout<<"\n";
#endif

             }
        }
        return moves.size();
}



// space reserved for TODO list
bool Vehicle::e_insertIntoFeasableTruck(const Trashnode &node,int pos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::e_insertIntoFeasableTruck \n";
#endif
	assert( feasable() ); 
	double localCost=cost;
     	if ( not path.e__insert(node,pos,maxcapacity) ) {
	        assert( feasable() );
		return false;
	}
     	evalLast();

	if (not feasable() ) {
		path.e_remove(pos,maxcapacity);
     		evalLast();
		assert(localCost == cost);
	        assert( feasable() );
		return false;
        };

	assert( feasable() );
	return true;
}

//dont forget, negative savings is a higher cost
bool Vehicle::eval_erase(int at, double &savings) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_erase \n";
#endif
	assert (at<size() and at>0 );
	if ( path[at].isdump() ) { savings=_MIN(); return false;}
	Vehicle truck = (*this);
	truck.path.erase(at);
	if ( not truck.e_makeFeasable(at) ) savings = _MIN(); // -infinity
        else savings = cost - truck.cost;

	return truck.feasable();
};
	
//dont forget, negative savings is a higher cost
bool Vehicle::eval_erase(int at, double &savings,const TWC<Trashnode> &twc) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_erase \n";
#endif
        assert (at<size() and at>0 );
        if ( path[at].isdump() ) { savings=_MIN(); return false;}
        Vehicle truck = (*this);
	double oldcost=truck.getCost(twc);

        truck.path.erase(at);

        if ( not truck.e_makeFeasable(at) ) savings = _MIN(); // -infinity
        else savings = oldcost - truck.getCost(twc);

#ifdef TESTED
std::cout<<"ERASE : oldcost"<<oldcost<<"\tnewcost"<<truck.getCost(twc)<<"\tsavings"<<oldcost - truck.getCost(twc)<<"\n";
std::cout<<"\n";
#endif
        return truck.feasable();
};

long int Vehicle::eval_insertMoveDumps( const Trashnode &node,Moves &moves, int fromTruck, int fromPos, int toTruck, double eraseSavings, double factor, const TWC<Trashnode> &twc) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_insertMoveDumps \n";
#endif
        Vehicle truck = (*this);
        std::deque<int> unTestedPos;
        std::deque<int> unfeasablePos;
        std::deque<int> impossiblePos;
        int currentPos,testingPos;
	double oldcost=truck.getCost(twc);
	double newcost;
#ifdef TESTED
truck.dumpCostValues();
#endif

        for ( int i=1; i<=size(); i++) unTestedPos.push_back(i);
        while (unTestedPos.size()) {
             currentPos=unTestedPos.back();
             unTestedPos.pop_back();
             truck.insert(node,currentPos);
	
             if ( not truck.e_makeFeasable(currentPos) ) {
#ifdef TESTED
truck.tau();
truck.dumpeval();
std::cout<<"\n";
assert(true==false);
#endif
                impossiblePos.push_back(currentPos);
                if ( path.size()*factor > impossiblePos.size() ) return moves.size();
             } else {
                assert ( truck.feasable() );
	        newcost=truck.getCost(twc);
#ifdef TESTED
std::cout<<"insert to "<<toTruck<<": oldcost"<<oldcost<<"\tnewcost"<<truck.getCost(twc)
	<<"\ninsert savings="<< (oldcost-newcost) <<"\teraseSavings"<<eraseSavings<<"\tsavings"<<oldcost - newcost + eraseSavings<<"\n";
std::cout<<"\n";
#endif
                Move move(Move::Ins, node.getnid(),  -1,  fromTruck, toTruck,  fromPos, currentPos, (oldcost - newcost + eraseSavings)   );
                moves.insert(move);
#ifdef TESTED
move.dump();
#endif
             }
             truck=(*this);
        }
        return moves.size();
}
	
	

long int Vehicle::eval_insertMoveDumps( const Trashnode &node,std::deque<Move> &moves, int fromTruck, int fromPos, int toTruck, double eraseSavings, double factor) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_insertMoveDumps \n";
#endif
	Vehicle truck = (*this);
	std::deque<int> unTestedPos;
	std::deque<int> unfeasablePos;
	std::deque<int> impossiblePos;
	int currentPos,testingPos;



        for ( int i=1; i<=size(); i++) unTestedPos.push_back(i); 
        while (unTestedPos.size()) {
             currentPos=unTestedPos.back();
	     unTestedPos.pop_back();
	     truck.insert(node,currentPos);
             if ( not truck.e_makeFeasable(currentPos) ) {
		impossiblePos.push_back(currentPos);
                if ( path.size()*factor > impossiblePos.size() ) return moves.size(); 
             } else {
		assert ( truck.feasable() );
		Move move(Move::Ins, node.getnid(),  -1,  fromTruck, toTruck,  fromPos, currentPos, (cost-truck.cost+eraseSavings)   );
		moves.push_back(move);

                truck.remove(currentPos);
		//unknown state of truck here
                while ( unTestedPos.size()>0 ) {
                   testingPos= unTestedPos.back();
	           unTestedPos.pop_back();
		   if ( testingPos<path.size() and  path[ testingPos ].isdump()) continue; //skipping dumps
        	   if ( truck.e_insertIntoFeasableTruck( node, testingPos) ) {
			Move move(Move::Ins, node.getnid(),  -1,  fromTruck, toTruck,  fromPos, testingPos, (cost-truck.cost + eraseSavings)   );
			moves.push_back(move);
                   } else unfeasablePos.push_back( testingPos);
		   truck.remove( testingPos );
		}
		unTestedPos=unfeasablePos;
             }
             truck=(*this);
        }
	return moves.size();
}

bool Vehicle::e_makeFeasable(int currentPos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::e_makeFeasable\n";
#endif
    path.e__adjustDumpsToMaxCapacity(currentPos, dumpSite, maxcapacity);
    evalLast();
    return feasable();
}

bool Vehicle::applyMoveINSerasePart(int nodeNid, int pos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::applyMoveINSerasePart\n";
#endif
	assert (path[pos].getnid()==nodeNid); //if this assertion fails might be because its not being applied to the correct solution
	if (not (path[pos].getnid()==nodeNid))  return false;
        path.erase(pos);
        e_makeFeasable( pos );
if (not feasable() ) dumpeval();
        assert ( feasable() );
        return feasable();
}


bool Vehicle::applyMoveINSinsertPart(const Trashnode &node, int pos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::applyMoveINSinsertPart\n";
#endif
	path.insert(node,pos);
	e_makeFeasable( pos );
if (not feasable() ) dumpeval();
	assert ( feasable() );
	return feasable();
}

bool Vehicle::applyMoveInterSw(Vehicle &otherTruck,int truckPos, int otherTruckPos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::applyMoveIntraSw\n";
#endif
	path.swap( truckPos,  otherTruck.path, otherTruckPos);
        e_makeFeasable( truckPos );
        otherTruck.e_makeFeasable( otherTruckPos );

        //evalLast();
        //otherTruck.evalLast();

        assert ( feasable() );
        assert ( otherTruck.feasable() );
        return feasable() and otherTruck.feasable();
}

bool Vehicle::applyMoveIntraSw(int  fromPos, int withPos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::applyMoveInterSw\n";
#endif
        path.swap( fromPos,  withPos);
        if (not e_makeFeasable( std::min(fromPos-1,withPos) ) ) return false;
        //evalLast(); done in makeFeasable
        assert ( feasable() );
        return feasable() ;
}
	


bool Vehicle::e_insertMoveDumps( const Trashnode &node, int at) {
	assert (at<=size());
//
//        path.insert(node,at);
//        path.e_moveDumps(at);
//
}



// Very TIGHT insertion 
    // insertion will not be performed if 
    //      TV and CV are  generated 
    //  true- insertion was done
    //  false- not inserted 
bool Vehicle::e_insertSteadyDumpsTight(const Trashnode &node, int at){
    assert ( at<=size() );
#ifndef TESTED
std::cout<<"Entering Vehicle::e_insertSteadyDumpsTight \n";
#endif


    if ( deltaCargoGeneratesCV(node,at) ) return false;
    if ( deltaTimeGeneratesTV(node,at) ) return false;
path[size()-1].dumpeval();
    if ( path.e_insert(node,at,maxcapacity) ) return false;
    evalLast();

    assert ( feasable() );
    return true;
};


// end space reserved for TODO list


bool Vehicle::e_insertDumpInPath( const Trashnode &lonelyNodeAfterDump ) {
#ifndef TESTED
std::cout<<"Entering Vehicle::e_insertDumpInPath \n";
#endif
    //we arrived here because of CV
    if ( deltaTimeGeneratesTV( dumpSite,lonelyNodeAfterDump ) ) return false;
    Trashnode dump=dumpSite;
    dump.setDemand(-getcargo());
    path.e_push_back(dump,maxcapacity);
    path.e_push_back(lonelyNodeAfterDump,maxcapacity);
    evalLast();

    assert ( feasable() );
    return true;
};
    

    




//bool Vehicle::deltaCargoGeneratesCV_AUTO(const Trashnode &node, int pos) const { //position becomes important

bool Vehicle::deltaCargoGeneratesCV(const Trashnode &node, int pos) const { //position becomes important
#ifdef TESTED
std::cout<<"Entering Vehicle::deltaCargoGeneratesCV \n";
//std::cout<<getcargo()<<"+"<<node.getdemand()<<" ¿? " <<getmaxcapacity()<<" \n";
#endif
     //cycle until a dump is found
     int i;
     for (i=pos; i<size() and not isdump(i); i++) {};
     // two choices i points to a dump or i == size() 
     // in any case the i-1 node has the truck's cargo
#ifdef TESTED
path[i-1].dumpeval();
std::cout<<getCargo(i-1)<<"+"<<node.getdemand()<<" ¿? " <<getmaxcapacity()<<" \n";
#endif
     return  ( path[i-1].getcargo() + node.getdemand() > maxcapacity  ) ;
};




//////////// Delta Time generates TV
bool Vehicle::deltaTimeGeneratesTV(const Trashnode &dump, const Trashnode &node) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::deltaTimeGeneratesTV  ";
std::cout<<" (S 1 2 3 D E )  (S 1 2 3 D N D E)"<<path.getDeltaTimeAfterDump(dumpSite,node)<<" + "<< getduration()<<" ¿? "<<  endingSite.closes();
std::cout<<"\n";
#endif
     return  ( path.getDeltaTimeAfterDump(dumpSite,node) + getduration()  > endingSite.closes() ) ;
}





bool Vehicle::deltaTimeGeneratesTV(const Trashnode &node, int pos) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::deltaTimeGeneratesTV \n";
if (pos>path.size()) std::cout<<"CANT work with this pos:"<<pos<<"\n";
std::cout<<"\n";
if (pos==path.size()) std::cout<<" (S 1 2 3 D E )  (S 1 2 3 N D E)" << path.getDeltaTime(node,dumpSite)<<" + "<< getduration()<<" ¿? "<<  endingSite.closes()<<"\n";
else std::cout<<" (S 1 2 3 D E )  (S 1 2 N 3 D E) "<< path.getDeltaTime(node,pos)<<" + "<< getduration()<<" ¿? "<<  endingSite.closes()<<"\n";
std::cout<<"\n";
endingSite.dump();
#endif
     assert(pos<=path.size());
     if (pos==path.size()) return path.getDeltaTime(node,dumpSite) + getduration()  > endingSite.closes();
     else return  ( path.getDeltaTime(node,pos) + getduration()  > endingSite.closes() ) ;
}
//////////////

