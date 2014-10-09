

#include <iostream>
#include <sstream>
#include <deque>

#include "trashconfig.h"
#include "twpath.h"
#include "vehicle.h"
#include "osrm.h"
#include "move.h"

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


long int Vehicle::eval_insertMoveDumps( const Trashnode &node,std::deque<Move> &moves, int fromTruck, int fromPos, int toTruck, double factor) const {
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
		Move move(Move::Ins, node.getnid(),  -1,  fromTruck, toTruck,  fromPos, currentPos, (cost-truck.cost)   );
		moves.push_back(move);

                truck.remove(currentPos);
		//unknown state of truck here
                while ( unTestedPos.size()>0 ) {
                   testingPos= unTestedPos.back();
	           unTestedPos.pop_back();
		   if ( testingPos<path.size() and  path[ testingPos ].isdump()) continue; //skipping dumps
        	   if ( truck.e_insertIntoFeasableTruck( node, testingPos) ) {
			Move move(Move::Ins, node.getnid(),  -1,  fromTruck, toTruck,  fromPos, testingPos, (cost-truck.cost)   );
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

bool Vehicle::applyMoveINS(const Trashnode &node, int pos) {
#ifndef TESTED
std::cout<<"Entering Vehicle::applyMoveINS\n";
#endif
	insert(node,pos);
	e_makeFeasable( pos );
	evalLast();
	assert ( feasable() );
	return feasable();
}
	


bool Vehicle::e_insertMoveDumps( const Trashnode &node, int at) {
	assert (at<=size());
/*
        path.insert(node,at);
        path.e_moveDumps(at);
*/
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
    

    



// getTimeOSRM() REQUIRES the main() to call cURLpp::Cleanup myCleanup; ONCE!

double Vehicle::getTimeOSRM() const {
    std::ostringstream url(std::ostringstream::ate);
    url.str(CONFIG->osrmBaseUrl);
    url << "viaroute?z=18&instructions=false&alt=false";

    OSRM osrm;
    int status;
    double ttime;

    for (int i=0; i<path.size(); i++)
        url << "&loc=" << path[i].gety() << "," << path[i].getx();

    url << "&loc=" << dumpSite.gety() << "," << dumpSite.getx();
    url << "&loc=" << endingSite.gety() << "," << endingSite.getx();

//std::cout << "OSRM: " << url.str() << std::endl;

    if(osrm.callOSRM(url.str())) {
        std::cout << "osrm.callOSRM: failed for url: " << url << std::endl;
        return -1.0;
    }

    if(osrm.getStatus(status)) {
        std::cout << "osrm.getStatus: reported: " << status << std::endl;
        return -1.0;
    }

    if(osrm.getTravelTime(ttime)) {
        std::cout << "osrm.getTravelTime failed to find the travel time!" << std::endl;
        return -1.0;
    }

    ttime += path.getTotWaitTime() + path.getTotServiceTime();

    return ttime;
}

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
#ifndef TESTED
std::cout<<"Entering Vehicle::deltaTimeGeneratesTV  ";
std::cout<<" (S 1 2 3 D E )  (S 1 2 3 D N D E)"<<path.getDeltaTimeAfterDump(dumpSite,node)<<" + "<< getduration()<<" ¿? "<<  endingSite.closes();
std::cout<<"\n";
#endif
     return  ( path.getDeltaTimeAfterDump(dumpSite,node) + getduration()  > endingSite.closes() ) ;
}





bool Vehicle::deltaTimeGeneratesTV(const Trashnode &node, int pos) const {
#ifndef TESTED
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




bool Vehicle::e_setPath(const Bucket &sol) {
#ifdef TESTED
std::cout<<"Entering Vehicle::e_setPath \n";
#endif
     assert (sol.size());
     if (not sol.size()) return false;

     if (not ( (sol[0]==path[0]) and (sol[sol.size()-1] == endingSite) and (sol[sol.size()-2] == dumpSite) ) ) 
         return false;
     path=sol;
     path.pop_back();
     path.pop_back();
     path[0].evaluate( getmaxcapacity() );
     path.evaluate(1 , getmaxcapacity());

     assert((sol[0]==path[0]) and (sol[sol.size()-1] == endingSite) and (sol[sol.size()-2] == dumpSite));
     return true;
}     



bool  Vehicle::findNearestNodeTo(Bucket &unassigned, const TWC<Trashnode> &twc,UID &pos, Trashnode &bestNode) {
#ifdef TESTED
std::cout<<"Entering Vehicle::findNearestNodeTo \n";
#endif
    assert (unassigned.size());
    if (not unassigned.size()) return false;

    bool flag= false;
    double bestDist;
    double d;
    
    flag = twc.findNearestNodeTo(path, unassigned,  pos , bestNode, bestDist);
    
    for (int i=0; i<unassigned.size(); i++) {
       if ( twc.isCompatibleIAJ( path[size()-1]  , unassigned[i], dumpSite ) ) { 
          d = unassigned[i].distanceToSegment( path[size()-1], dumpSite );
          if ( d < bestDist) {
            bestDist = d;
            bestNode = unassigned[i];
            pos = size();
            flag= true;
          };
       };
    }

    return flag;
}

























//   Steve's functions 

void Vehicle::dump() const {
    std::cout << "---------- Vehicle ---------------" << std::endl;
    std::cout << "maxcapacity: " << getmaxcapacity() << std::endl;
    std::cout << "cargo: " << getcargo() << std::endl;
    std::cout << "duration: " << getduration() << std::endl;
    std::cout << "cost: " << getcost() << std::endl;
    std::cout << "OSRM time: " << getTimeOSRM() << std::endl;
    std::cout << "TWV: " << getTWV() << std::endl;
    std::cout << "CV: " << getCV() << std::endl;
    std::cout << "w1: " << getw1() << std::endl;
    std::cout << "w2: " << getw2() << std::endl;
    std::cout << "w3: " << getw3() << std::endl;
    std::cout << "path nodes: -----------------" << std::endl;
    path.dump();
}


void Vehicle::dumppath() const {
    path.dump();
}

    void Vehicle::dump(const std::string &title) const {
       path.dump(title);
    }


   void Vehicle::dumpeval() const {
     std::cout<<"\nStarting site:"<<"\n";
     path[0].dumpeval();
     for (int i=1;i<path.size();i++){
         std::cout<<"\npath stop #:"<<i<<"\n";
          path[i].dumpeval();
     }
     std::cout<<"\nDump site:"<<"\n";
     dumpSite.dumpeval();
     std::cout<<"\nEnding site :"<<"\n";
     endingSite.dumpeval();
     std::cout <<"TOTAL COST="<<cost <<"\n";
   }


   void Vehicle::smalldump() const {
      std::cout<<"\nDump site:"<<"\n";
      dumpSite.dumpeval();
      std::cout<<"\nEnding site :"<<"\n";
      endingSite.dumpeval();
      std::cout <<"TOTAL COST="<<cost <<"\n";
   }

   void Vehicle::tau() const {
      for (int i=0; i< path.size(); i++)
         std::cout<<getnid(i)<<" , ";
      std::cout<<dumpSite.getnid()<<" , ";
      std::cout<<endingSite.getnid()<<" , ";
      std::cout<<" (";
      for (int i=0; i< path.size(); i++)
         std::cout<<getid(i)<<" , ";
      std::cout<<dumpSite.getid()<<" , ";
      std::cout<<endingSite.getid()<<" , ";
      std::cout<<" )";
   }


std::deque<int> Vehicle::getpath() const {
      std::deque<int> p;
      p = path.getpath();
      p.push_front(getdepot().getnid());
      p.push_back(getdumpSite().getnid());
      p.push_back(getdepot().getnid());
      return p;
}


bool Vehicle::push_back(Trashnode node) {
    assert (node.getnid()>=0);
    E_Ret ret = path.e_push_back(node, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::push_front(Trashnode node) {
    // position 0 is the depot we can not put a node before that
    return insert(node, 1);
}


bool Vehicle::insert(Trashnode node, int at) {
    E_Ret ret = path.e_insert(node, at, getmaxcapacity());
    if (ret == OK) {
        path.evaluate(at, getmaxcapacity());
        evalLast();
    }
    else if (ret == INVALID) return false;
    return true;
}

bool Vehicle::remove(int at) {
    E_Ret ret = path.e_remove(at, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::moverange( int rangefrom, int rangeto, int destbefore ) {
    E_Ret ret = path.e_move(rangefrom, rangeto, destbefore, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::movereverse( int rangefrom, int rangeto, int destbefore ) {
    E_Ret ret = path.e_movereverse(rangefrom, rangeto, destbefore, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::reverse( int rangefrom, int rangeto ) {
    E_Ret ret = path.e_reverse(rangefrom, rangeto, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::move( int fromi, int toj ) {
    E_Ret ret = path.e_move(fromi, toj, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::swap( const int& i, const int& j ) {
    E_Ret ret = path.e_swap(i, j, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Vehicle::swap(Vehicle& v2, const int& i1, const int& i2) {
    E_Ret ret = path.e_swap(i1, getmaxcapacity(),
                    v2.getvpath(), i2, v2.getmaxcapacity());
    if (ret == OK) {
        evalLast();
        v2.evalLast();
    }
    else if (ret == INVALID) return false;
    return true;
}


void Vehicle::restorePath(Twpath<Trashnode> oldpath) {
    path = oldpath;
    evalLast();
}


void Vehicle::evalLast() {
    Trashnode last = path[path.size()-1];
    dumpSite.setDemand(-last.getcargo());
    dumpSite.evaluate(last, getmaxcapacity());
    endingSite.evaluate(dumpSite, getmaxcapacity());
    cost = w1*endingSite.getTotTime() +
           w2*endingSite.getcvTot() +
           w3*endingSite.gettwvTot();
}


//--------------------------------------------------------------------------
// intra-route optimiziation
//--------------------------------------------------------------------------

bool Vehicle::doTwoOpt(const int& c1, const int& c2, const int& c3, const int& c4) {
    // Feasible exchanges only
    if ( c3 == c1 || c3 == c2 || c4 == c1 || c4 == c2 || c2 < 1 || c3 < 2 )
        return false;

    double oldcost = getcost();

    // Leave values at positions c1, c4
    // Swap c2, c3
    // c3 -> c2
    // c2 -> c3
    // reverse any nodes between c2 and c3
    // this reduces to just a simple reverse(c2, c3)
    if (! reverse(c2, c3)) return false;

    // if the change does NOT improve the cost or generates TW violations
    // undo the change
    if (getcost() > oldcost or hastwv()) {
        reverse(c2, c3);
        return false;
    }

    return true;
}


bool Vehicle::doThreeOpt(const int& c1, const int& c2, const int& c3, const int& c4, const int& c5, const int& c6) {
    // Feasible exchanges only
    if (! (c2>c1 && c3>c2 && c4>c3 && c5>c4 && c6>c5)) return false;

    double oldcost = getcost();
    Twpath<Trashnode> oldpath(path); // save a copy for undo

    // the 3-opt appears to reduce to extracting a sequence of nodes c3-c4
    // and reversing them and inserting them back after c6
    if (! movereverse(c2, c3, c6)) return false;

    if (getcost() > oldcost or hastwv()) {
        restorePath(oldpath);
        return false;
    }

    return true;
}


bool Vehicle::doOrOpt(const int& c1, const int& c2, const int& c3) {
    // Feasible exchanges only
    if (! (c2 >= c1 and (c3 < c1-1 or c3 > c2+2))) return false;
    if (c2 > path.size()-1 or c3 > path.size()-1) return false;

    double oldcost = getcost();
    Twpath<Trashnode> oldpath(path); // save a copy for undo

    if (! moverange(c1, c2, c3)) return false;

    if (getcost() > oldcost or hastwv()) {
        restorePath(oldpath);
        return false;
    }

    return true;
}


bool Vehicle::doNodeMove(const int& i, const int& j) {
    if (i == j or i < 1 or j < 1 or i > path.size()-1 or j > path.size()-1)
        return false;

    double oldcost = getcost();

    if (! move(i, j)) return false;

    if (getcost() > oldcost or hastwv()) {
        move(j, i);
        return false;
    }

    return true;
}


bool Vehicle::doNodeSwap(const int& i, const int& j) {
    if (i < 1 or j < 1 or i > path.size()-1 or j > path.size()-1)
        return false;

    double oldcost = getcost();

    if (! swap(i, j)) return false;

    if (getcost() > oldcost or hastwv()) {
        swap(i, j);
        return false;
    }

    return true;
}


bool Vehicle::doInvertSeq(const int& i, const int& j) {
    if (i > path.size() or j > path.size()-1)
        return false;

    double oldcost = getcost();

    if (! reverse(i, j)) return false;

    if (getcost() > oldcost or hastwv()) {
        reverse(i, j);
        return false;
    }

    return true;
}


bool Vehicle::pathOptimize() {
    // repeat each move until the is no improvement then move to the next
    bool improvement = false;

    // 1. move node forward
    // 2. move node down
    improvement = improvement or pathOptMoveNodes();

    // 3. 2-exchange
    improvement = improvement or pathOptExchangeNodes();

    // 4. sequence invert
    improvement = improvement or pathOptInvertSequence();

    return improvement;
}


bool Vehicle::pathTwoOpt() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    do {
        oldcost = getcost();

        for (int i=0; i<size-3; i++) {
            for (int j=i+2; j<size; j++) {
                doTwoOpt( i, i+1, j, j+1 );
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathThreeOpt() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    do {
        oldcost = getcost();

        for (int i=0; i<size-5; i++) {
            for (int j=i+2; j<size-3; j++) {
                for (int k=j+2; k<size-1; k++) {
                    doThreeOpt( i, i+1, j, j+1, k, k+1 );
//                    std::cout << "pathThreeOpt["<<i<<","<<i+1<<","<<j<<","<<j+1<<","<<k<<","<<k+1<<"]("<<getcost()<<"): ";
//                    dumppath();
                }
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOrOpt() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=1; j<size; j++) {
                for (int k=2; k>=0; k--) {
                    if (! (j<i-1 or j>i+k+1)) continue;
                    doOrOpt( i, i+k, j );
//std::cout << "pathOrOpt["<<i<<","<<i+k<<","<<j<<"]("<<getcost()<<"): ";
//dumppath();
                }
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOptMoveNodes() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    // move the nodes forward looking for improvements
    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=i+1; j<size; j++) {
                doNodeMove(i, j);
            }
        }
    }
    while (getcost() < oldcost);

    // move the nodes backwards looking for improvements
    do {
        oldcost = getcost();

        for (int i=size-1; i>0; i--) {
            for (int j=i-1; j>0; j--) {
                doNodeMove(i, j);
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOptExchangeNodes() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    // 2-exchange (swapping) nodes along the path
    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=i+1; j<size; j++) {
                doNodeSwap(i, j);
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


bool Vehicle::pathOptInvertSequence() {
    int size = this->size();

    double origcost = getcost();
    double oldcost;

    // invert all possible sequences of nodes
    do {
        oldcost = getcost();

        for (int i=1; i<size; i++) {
            for (int j=i+1; j<size; j++) {
                doInvertSeq(i, j);
            }
        }
    }
    while (getcost() < oldcost);

    return getcost() < origcost;
}


// find the best place to add tn into this route based on minimizing
// the increase in cost of the route to add it

// TODO: change this to check all move ops for failure
//       and restore the original paths
// it currently assume all ops succeed but if they dont
// the paths will get realy messed up

bool Vehicle::findBestFit(const Trashnode& tn, int* tpos, double* deltacost) {
    int bestpos = -1;
    double bestdelta;

    double origcost = getcost();

    // first we insert nid into the start of vp
    // ie: pos: 1 after the depot at pos: 0
    if (! insert(tn, 1)) return false;

    if (feasable()) {
        bestpos = 1;
        bestdelta = getcost() - origcost;
    }

    // now we walk it down the path checking for better costs
    for (int i=2; i<this->size(); i++) {
        swap(i-1, i);
        if (getcost() - origcost < bestdelta and feasable()) {
            bestpos = i;
            bestdelta = getcost() - origcost;
        }
    }

    // this is only a test so remove the node
    remove(this->size()-1);

    // if we found NO feasable place to insert it
    if (bestpos == -1) {
        return false;
    }

    *tpos = bestpos;
    *deltacost = bestdelta;
    return true;
}


// --------------------------------------------------------------------------
// Inter-route modifications
// --------------------------------------------------------------------------

/*
    2-exchange - swap path[i1] with v2[i2]
*/
bool Vehicle::swap2(Vehicle& v2, const int& i1, const int& i2, bool force) {
    if (i1 < 0 or i1 > this->size()-1 or i2 < 0 or i2 > v2.size()-1)
        return false;

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    if (! swap(v2, i1, i2)) return false;

    if (force) return true;

    double newcost1 = getcost();
    double newcost2 = v2.getcost();

    if ( newcost1 + newcost2 > oldcost1 + oldcost2 or
         hastwv() or hascv() or v2.hastwv() or v2.hascv() ) {
        swap(v2, i1, i2);

        return false;
    }

    return true;
}


/*
    3-route node exchange - swap3
    path[i1] -> v2.path[i2] -> v3.path[i3] -> path[i1]
*/
bool Vehicle::swap3(Vehicle& v2, Vehicle& v3, const int& i1, const int& i2, const int& i3, bool force) {
    if ( i1 < 0 or i1 > this->size()-1 or
         i2 < 0 or i2 > v2.size()-1 or
         i2 < 0 or i3 > v3.size()-1 ) return false;

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();
    double oldcost3 = v3.getcost();

    // this require all both swaps to work
    // if the 1st fails we can abort and return false
    // if the 2nd fails we have to unwind the 1st then abort
    if (! swap(v2, i1, i2)) return false;
    if (! v2.swap(v3, i2, i3)) {
        swap(v2, i1, i2);
        return false;
    }

    if (force) return true;

    double newcost1 = getcost();
    double newcost2 = v2.getcost();
    double newcost3 = v3.getcost();

    if ( newcost1 + newcost2 + newcost3 > oldcost1 + oldcost2 + oldcost3 or
         !feasable() or !v2.feasable() or !v3.feasable() ) {
        v2.swap(v3, i2, i3);
        swap(v2, i1, i2);
        return false;
    }

    return true;
}



//  exchange seq - exchange
//  exchange a sequence of nodes between two routes
//  path[i1..j1] <--> v2.path[i2..j2]

// TODO: convert this to use non-evaluating functions
//       and then just evaluate the whole path when done

bool Vehicle::exchangeSeq(Vehicle& v2, const int& i1, const int& j1, const int& i2, const int& j2, bool force) {
    if ( j1 < i1 or j2 < i2 or i1 < 0 or i2 < 0 or
         j1 > this->size()-1 or j2 > v2.size()-1 ) return false;

    const Twpath<Trashnode> p1 = getvpath();      // get a copy
    const Twpath<Trashnode> p2 = v2.getvpath();   // get a copy

    Twpath<Trashnode>& v2p = v2.getvpath(); // get a reference to manipulate

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    int d1 = j1-i1+1;
    int d2 = j2-i2+1;

    // first swap the nodes in the min length of the seq length
    for (int n=0; n<std::min(d1, d2); n++) {
        path.e_swap(i1+n, getmaxcapacity(),
            v2p, i2+n, v2.getmaxcapacity());
    }

    // now if the seqs were different lengths move the remainder
    if (d1 > d2) {
        for (int n=0; n<d1-d2; n++) {
            if (i2+d2+n >= v2.size())
                v2p.e_push_back(path[i1+d2], v2.getmaxcapacity());
            else
                v2p.e_insert(path[i1+d2], i2+d2+n, v2.getmaxcapacity());
            path.e_remove(i1+d2, getmaxcapacity());
        }
    }
    else if (d2 > d1) {
        for (int n=0; n<d2-d1; n++) {
            if (i1+d1+n >= this->size())
                path.e_push_back(v2p[i2+d1], getmaxcapacity());
            else
                path.e_insert(v2p[i2+d1], i1+d1+n, getmaxcapacity());
            v2p.e_remove(i2+d1, v2.getmaxcapacity());
        }
    }

    evalLast();
    v2.evalLast();

    if (force) return true;

    double newcost1 = getcost();
    double newcost2 = v2.getcost();

    if ( newcost1 + newcost2 > oldcost1 + oldcost2 or
             !feasable() or !v2.feasable() ) {
        setvpath(p1);
        v2.setvpath(p2);
        evalLast();
        v2.evalLast();

        return false;
    }

    return true;
}


/*
    exchange tails
    this swaps the seq of nodes from an index to the end of the path with
    another path and its given index
    exchange v1[i1...n1] <--> v2[i2..n2]
*/
bool Vehicle::exchangeTails(Vehicle& v2, const int& i1, const int& i2, bool force) {
    if ( i1 < 0 or i1 > this->size()-1 or
         i2 < 0 or i2 > v2.size()-1 ) return false;

    return exchangeSeq(v2, i1, this->size()-1, i2, v2.size()-1, force);
}



//  exchange3
// this exchanges a sequence of cnt nodes starting at i1, i2 and i3
// between thre respective vehicles this*, v2, and v3
// the nodes in this* are moved to v2 and
// the nodes in v2 are moved to v3 and
// the nodes in v3 are moved to this*

// TODO: convert this to use non-evaluating twpath functions
//       and then just call evaluate on each

bool Vehicle::exchange3(Vehicle& v2, Vehicle& v3, const int& cnt, const int& i1, const int& i2, const int& i3, bool force) {
    if ( i1 < 0 or i1+cnt > this->size()-1 or
         i2 < 0 or i2+cnt > v2.size()-1 or
         i3 < 0 or i3+cnt > v3.size()-1 or cnt < 1) return false;

    Twpath<Trashnode>& v2p = v2.getvpath(); // get a reference to manipulate
    Twpath<Trashnode>& v3p = v3.getvpath(); // get a reference to manipulate

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();
    double oldcost3 = v3.getcost();

std::cout << "oldcost: "<<oldcost1<<"+"<<oldcost2<<"+"<<oldcost3<<"="
          << oldcost1 + oldcost2 + oldcost3 << std::endl; 

    for (int i=0; i<cnt; i++) {
        path.e_swap(i1+i, getmaxcapacity(),
            v2.getvpath(), i2+i, v2.getmaxcapacity());
        v2.getvpath().e_swap(i2+i, v2.getmaxcapacity(),
            v3.getvpath(), i3+i, v3.getmaxcapacity());
    }
    evalLast();
    v2.evalLast();
    v3.evalLast();

    if (force) return true;

    double newcost1 = getcost();
    double newcost2 = v2.getcost();
    double newcost3 = v3.getcost();

std::cout << "newcost: "<<newcost1<<"+"<<newcost2<<"+"<<newcost3<<"="
          << newcost1 + newcost2 + newcost3 << std::endl; 

    if ( newcost1 + newcost2 + newcost3 > oldcost1 + oldcost2 + oldcost3 or
         !feasable() or !v2.feasable() or !v3.feasable() ) {
        for (int i=0; i<cnt; i++) {
            v2.getvpath().e_swap(i2+i, v2.getmaxcapacity(),
                v3.getvpath(), i3+i, v3.getmaxcapacity());
            path.e_swap(i1+i, getmaxcapacity(),
                v2.getvpath(), i2+i, v2.getmaxcapacity());
        }
        evalLast();
        v2.evalLast();
        v3.evalLast();

        return false;
    }

    return true;
}



//  relocate
//  move node v1[i1] to another path v2[i2]
//  returns false if it fails to relocate the node to v2

bool Vehicle::relocate(Vehicle& v2, const int& i1, const int& i2, bool force) {
    if ( i1 < 0 or i1 > this->size()-1 or
         i2 < 0 or i2 > v2.size()-1 ) return false;


    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    if (! v2.insert(path[i1], i2)) return false;
    if (!remove(i1)) {
        v2.remove(i2);
        return false;
    }

    if (force) return true;

    double newcost1 = getcost();
    double newcost2 = v2.getcost();

    if ( newcost1 + newcost2 > oldcost1 + oldcost2 or
         !feasable() or !v2.feasable() ) {
        insert(v2.path[i2], i1);
        v2.remove(i2);
        return false;
    }
    return true;
}



//  relocateBest
//  move node v1[i1] to the best location in v2
//  returns false if it fails to insert it.

// TODO: change this to check all move ops for failure
//       and restore the original paths
// it currently assume all ops succeed but if they dont
// the paths will get realy messed up

bool Vehicle::relocateBest(Vehicle& v2, const int& i1) {
    if ( i1 < 0 or i1 > this->size()-1 ) return false;

    double oldcost1 = getcost();
    double oldcost2 = v2.getcost();

    int bestpos = -1;
    double bestcost;

    // first we insert v1[i1] into the start of v2
    // ie: pos: 1 after the depot at pos: 0
    if (! v2.insert(path[i1], 1)) return false;

    if (v2.feasable()) {
        bestpos = 1;
        bestcost = v2.getcost();
    }

    // now we walk it down the path checking for better costs
    for (int i=2; i<v2.size(); i++) {
        v2.swap(i-1, i);
        if (v2.getcost() < bestcost and v2.feasable()) {
            bestpos = i;
            bestcost = v2.getcost();
        }
    }

    // if we found NO feasable place to insert it
    // restore v2 and return
    if (bestpos == -1) {
        v2.remove(v2.size()-1);
        return false;
    }

    // otherwise remove i1
    remove(i1);

    // check that we have a better overall cost
    // and reposition i1 to bestpos
    if (getcost() + bestcost < oldcost1 + oldcost2) {
        if (bestpos != v2.size()-1) {
            v2.move(v2.size()-1, bestpos);
        }
    }
    // else restore everything and return false
    else {
        insert(v2[v2.size()-1], i1);
        v2.remove(v2.size()-1);
        return false;
    }

    return true;
}

/**************************************PLOT************************************/
void Vehicle::plot(std::string file,std::string title,int carnumber){
//std::cout<<"USING VEHICLE PLOT\n";
    Twpath<Trashnode> trace=path;
    trace.push_back(dumpSite);
    trace.push_back(endingSite);
trace.dumpid("Path");
    trace.pop_front();
    trace.pop_back();
    trace.pop_back();
    /** cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber */
    std::stringstream convert;
    convert << carnumber;
    std::string carnum = convert.str();
    std::string extra="vehicle"+carnum ;

    Plot<Trashnode> graph( trace );
    graph.setFile( CONFIG->plotDir+file+extra+".png" );
    graph.setTitle( title+extra );
    graph.drawInit();
    for (int i=0; i<trace.size(); i++){
        if (trace[i].ispickup())  {
             graph.drawPoint(trace[i], 0x0000ff, 9, true);
        } else if (trace[i].isdepot()) {
             graph.drawPoint(trace[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(trace[i], 0xff0000, 7, true);
        }
    }
    plot(graph,carnumber);
    graph.save();
}

void Vehicle::plot(Plot<Trashnode> graph, int carnumber){
//std::cout<<"USING VEHICLE PLOT  1\n";
    Twpath<Trashnode> trace=path;
    trace.push_back(dumpSite);
    trace.push_back(endingSite);
    graph.drawPath(trace,graph.makeColor(carnumber*10), 1, true);
}


