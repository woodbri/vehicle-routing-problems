

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

long int Vehicle::eval_intraSwapMoveDumps( std::deque<Move> &moves, int  truckPos, int fromPos,  double factor) const {
#ifdef TESTED
std::cout<<"Entering Vehicle::eval_intraSwapMoveDumps \n";
#endif
        if ( path[fromPos].isdump() ) return moves.size();
        double originalCost= cost;
        double newCost;
	double deltaTravelTime;
	double newCargoAtNearestFromPosDump,newCargoAtCurrentNearestDump ; 
	int moveDumpsFrom;
	int currentNearestDumpPos;

        Trashnode node = path[fromPos]; //saved for roll back
        Vehicle truck = (*this);
        std::deque<int> unTestedPos;
        std::deque<int> impossiblePos;
        std::deque<int> dumpsPos;
        int currentPos;
	Trashnode fromPosNearestDump;
        bool foundFromPosNearestDump=false;
	bool foundCurrentNearestDump;

        for ( int i=fromPos+1; i<size(); i++) {
                if ( not path[i].isdump() ) unTestedPos.push_back(i); //cant swap with a dump
		if (path[i].isdump() and not foundFromPosNearestDump) {
			dumpsPos.push_back(i);
			foundFromPosNearestDump=true;
		}
        }

	if (foundFromPosNearestDump==false){
		fromPosNearestDump = path[size()-1];   //the last node info can be used as the dumpfor CV
		fromPosNearestDump.setDemand( - fromPosNearestDump.getdemand() );
	} else fromPosNearestDump= path[ dumpsPos[0] ];

        while (unTestedPos.size()) {
             currentPos=unTestedPos.front();
             unTestedPos.pop_front();
	     deltaTravelTime = path.getDeltaTimeSwap( fromPos, currentPos) ;
	     if (deltaTravelTime==_MAX()) {
		impossiblePos.push_back(currentPos);
		continue;
	     };
		//no TWV now to check for dump moving
	     if ( path[fromPos].getdemand()  == path[currentPos].getdemand() or foundFromPosNearestDump==false) { //no need to move dumps

                	Move move(Move::IntraSw , node.getnid(), path[currentPos].getnid() ,  truckPos , truckPos ,  fromPos, currentPos, (- deltaTravelTime )   );
                	moves.push_back(move);
#ifndef TESTED
if (node.getnid()== 138 or node.getnid()== 148) {
move.dump();
std::cout<<"origina cost"<<originalCost<<"\t deltaTravelTime "<<  deltaTravelTime;
std::cout<<"\n";
    std::stringstream convert;
    convert << currentPos<<" swaping"<<path[currentPos].getnid()<<"with"<<node.getnid();
    std::string carnum = convert.str();
truck=(*this);
truck.plot(carnum+"before","bintraSwp",currentPos);
truck.applyMoveIntraSw(fromPos,currentPos);
truck.plot(carnum+"zafter","zintraSwp",currentPos);
}
#endif
			continue;
	     };
		
	     if ( currentPos < dumpsPos[0] ) { //they share the same dump, no need to move dumps
                	Move move(Move::IntraSw , node.getnid(), path[currentPos].getnid() ,  truckPos , truckPos ,  fromPos, currentPos, (originalCost-newCost)   );
                	moves.push_back(move);
#ifdef TESTED
move.dump();
std::cout<<"origina cost"<<originalCost<<"\t new cost"<< newCost;
std::cout<<"\n";
#endif
			continue;
             }

	     newCargoAtNearestFromPosDump = fromPosNearestDump.getdemand() - path[currentPos].getdemand() + node.getdemand();
	     moveDumpsFrom =-1;

	     if ( - newCargoAtNearestFromPosDump>maxcapacity) moveDumpsFrom=fromPos;
	     else {
                foundCurrentNearestDump=false;
		for ( int i=1;i < dumpsPos.size(); i++) {
                    if (currentPos < dumpsPos[i] and not foundFromPosNearestDump) {
                        currentNearestDumpPos=i;
                        foundCurrentNearestDump=true;
			break;
                    }
		}
		if ( foundCurrentNearestDump )
			newCargoAtCurrentNearestDump = path[currentNearestDumpPos].getdemand() - node.getdemand() + path[currentPos].getdemand();
		else 
			newCargoAtCurrentNearestDump = - path[size()-1].getcargo() - node.getdemand() + path[currentPos].getdemand();
		if (newCargoAtCurrentNearestDump > maxcapacity ) moveDumpsFrom=currentPos;
	     }

	     if (moveDumpsFrom==-1) { //no CV
                	Move move(Move::IntraSw , node.getnid(), path[currentPos].getnid() ,  truckPos , truckPos ,  fromPos, currentPos, (originalCost-newCost)   );
                	moves.push_back(move);
#ifdef TESTED
move.dump();
std::cout<<"origina cost"<<originalCost<<"\t new cost"<< newCost;
std::cout<<"\n";
#endif
			continue;
             }
	     //dump moving is requiered here
	     assert ("Vehicle::intraSwapMoveDumps  move dumps part need implementation"=="");

        }
        return moves.size();
}


// from the second truck point of view
long int Vehicle::eval_interSwapMoveDumps( std::deque<Move> &moves, const Vehicle &other,int  truckPos,int  otherTruckPos, int fromPos,  double factor) const {
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
        // evalLast(); done in make feasable
        assert ( feasable() );
        return feasable();
}


bool Vehicle::applyMoveINSinsertPart(const Trashnode &node, int pos) {
#ifdef TESTED
std::cout<<"Entering Vehicle::applyMoveINSinsertPart\n";
#endif
	path.insert(node,pos);
	e_makeFeasable( pos );
	//evalLast(); makeFeasable alredy does it
	assert ( feasable() );
	return feasable();
}

bool Vehicle::applyMoveInterSw(Vehicle &otherTruck,int truckPos, int otherTruckPos) {
#ifndef TESTED
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
#ifndef TESTED
std::cout<<"Entering Vehicle::applyMoveInterSw\n";
#endif
        path.swap( fromPos,  withPos);
        e_makeFeasable( std::min(fromPos,withPos) );
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
std::cout<<"Entering Vehicle::deltaTimeGeneratesTV ";
if (pos==path.size()) std::cout<<" (S 1 2 3 D E )  (S 1 2 3 N D E)" << path.getDeltaTime(node,dumpSite)<<" + "<< getduration()<<" ¿? "<<  endingSite.closes()<<"\n";
else std::cout<<" (S 1 2 3 D E )  (S 1 2 N 3 D E) "<< path.getDeltaTime(node,pos)<<" + "<< getduration()<<" ¿? "<<  endingSite.closes()<<"\n";
std::cout<<"\n";
endingSite.dump();
#endif
     if (pos==path.size()) return path.getDeltaTime(node,dumpSite) + getduration()  > endingSite.closes();
     else return  ( path.getDeltaTime(node,pos) + getduration()  > endingSite.closes() ) ;
}
//////////////

