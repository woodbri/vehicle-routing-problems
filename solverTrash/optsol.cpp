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


// apply valid and feasable move to the current solution (this)
void OptSol::applyMove(const Move& m)  {
    if (m.getsavings() < 0)
        STATS->inc("neg savings applied");
    switch (m.getmtype()) {
        case Move::Ins:
            {
                if (m.getsavings() < 0) STATS->inc("neg sav Ins applied");
#ifndef STEVE_OLD
                //aplyInsMove already does an asertion at the end
                applyInsMove( m );
                assert( fleet[m.getvid1()].feasable() ); //just in case
                assert( fleet[m.getvid2()].feasable() );
#else
                // Get a copy of the node we are going to remove
                // remove if from v1
                Vehicle& v1 = fleet[m.getvid1()];
                Trashnode n1 = v1[m.getpos1()];     // TODO can be ref?
                v1.remove(m.getpos1());
                // Get a reference to vid2
                Vehicle& v2 = fleet[m.getvid2()];
                // and insert n1 at the appropriate location
                v2.insert(n1, m.getpos2());
                assert( fleet[m.getvid1()].feasable() );
                assert( fleet[m.getvid2()].feasable() );
#endif
            }
            break;
        case Move::IntraSw:
            {
                if (m.getsavings() < 0) STATS->inc("neg sav IntraSw applied");
                //Vehicle& v1 = fleet[m.getvid1()];
                //v1.swap(m.getpos1(), m.getpos2());
                applyIntraSwMove( m );
                assert( fleet[m.getvid1()].feasable() );
            }
            break;
        case Move::InterSw:
            {
                if (m.getsavings() < 0) STATS->inc("neg sav InterSw applied");
                //Vehicle& v1 = fleet[m.getvid1()];
                //Vehicle& v2 = fleet[m.getvid2()];
                //v1.swap(v2, m.getpos1(), m.getpos2());
                applyInterSwMove( m );
                assert( fleet[m.getvid1()].feasable() );
                assert( fleet[m.getvid2()].feasable() );
            }
            break;
    }
    computeCosts();
}


// make or simulate making the move and check if the modified
// route(s) are feasable
bool OptSol::isNotFeasible(const Move& m) const {
    switch (m.getmtype()) {
        case Move::Ins:
            {
            // remove a node will not make the path infeasible
            // so we only need to check on inserting it

            // copy the vehicle and the node
            // so we can change it and then throw it away
            Vehicle v1 = fleet[m.getvid1()];
            assert( m.getvid2()<fleet.size() );
            Vehicle v2 = fleet[m.getvid2()];
            assert( m.getpos1()<v1.size() ); 
            Trashnode n1 = v1[m.getpos1()];
            if (not v2.insert(n1, m.getpos2())) return true;
            if (not v2.feasable()) return true;
            }
            break;
        case Move::IntraSw:
            {
            Vehicle v1 = fleet[m.getvid1()];
            if (not v1.swap(m.getpos1(), m.getpos2())) return true;
            if (not v1.feasable()) return true;
            }
            break;
        case Move::InterSw:
            {
            Vehicle v1 = fleet[m.getvid1()];
            Vehicle v2 = fleet[m.getvid2()];
            if (not v1.swap(v2, m.getpos1(), m.getpos2())) return true;
            if (not v1.feasable() or not v2.feasable()) return true;
            }
            break;
        default:
            return true;
    }
    return false;
}


// make or simulate making the move and return the savings
// that it will generate. This would be equivalent to 
// savings = oldsolution.getcost() - newsolution.getcost()
double OptSol::getMoveSavings(const Move& m)  const {
    // TODO: improve this, this is probably very inefficient
    // for example IF we combined the savings calc with isNotFeasible()
    // we can get the savings from the new paths we made.
    // we would want to v1.remove(m.getpos1) to get that cost

    double oldCost = getCost();

    // make a copy of the current solution
    // we dont what to modify this as we are const
    OptSol newsol = *this;
    newsol.applyMove(m);
    double newCost = newsol.getCost();

    return oldCost - newCost;
}


int OptSol::clearRelatedMoves(std::deque<Move>& moves, const Move& lastMove)  const {
    int cnt = 0;
    int kept = 0;

    // an invalid move will trigger clearing all moves
    // if the fleet is <4 then all moves will get cleared regardless
    if ( lastMove.getmtype() == Move::Invalid or 
         (lastMove.getmtype() != Move::IntraSw and fleet.size()<4) ) {
        cnt = moves.size();
        moves.clear();
    }
    else {

        // otherwise we just remove the moves that would be invalidated
        // by the last move when it was applied
        std::deque<Move>::iterator it = moves.begin();
        while (it != moves.end()) {
            if ( it->getvid1() == lastMove.getvid1() ||
                 it->getvid1() == lastMove.getvid2() ||
                 it->getvid2() == lastMove.getvid1() ||
                 it->getvid2() == lastMove.getvid2() ) {
                ++cnt;
                it = moves.erase( it );
            }
            else {
                ++it;
                ++kept;
            }
        }
    }

std::cout << "clearRelatedMoves: cleared: " << cnt << ", kept: " << kept << std::endl;

    return cnt;
}


int OptSol::addRelatedMovesIns(std::deque<Move>& moves, const Move& lastMove)  const {
    int cnt = 0;
    bool all = lastMove.getmtype() == Move::Invalid;
    int va = lastMove.getvid1();
    int vb = lastMove.getvid2();

    // generate moves based on the list of vid positions
    // and add them to the moves container
    // iterate through the vechicles (vi, vj)
    for (int vi=0; vi<fleet.size(); vi++) {
        for (int vj=0; vj<fleet.size(); vj++) {
            if (vi==vj) continue;
            // assume we have vehicles 0-10
            // and lastMove has vid1=va and vid2=vb
            // then we want to renerate moves where
            //  vi      vj
            //------------
            //  va  -> 0-10
            //  vb  -> 0-10
            // 0-10 ->  va
            // 0-10 ->  vb
            // unless we need to generate all moves
            //
            if ( not all and
                 not ( (vi == va or vi == vb) or
                       (vj == va or vj == vb) ) ) continue;

            // iterate through the positions of each path (pi, pj)
            // dont exchange the depot in position 0
            for (int pi=1; pi<fleet[vi].size(); pi++) {

                // dont try to move the dump
                if(fleet[vi][pi].isdump()) continue;

                // allow the node to be inserted at the end of the route also
                // hence <=
                for (int pj=1; pj<=fleet[vj].size(); pj++) {

                    // we can't break because we might have multiple dumps
                    // if we could get the position of the next dump
                    // we could jump pj forward to it
                    if (fleet[vj].deltaCargoGeneratesCV(fleet[vi][pi], pj))
                        continue;

                    // if we check TWC we can break out if pj/->pi
                    if (fleet[vj].deltaTimeGeneratesTV(fleet[vi][pi], pj))
                        break;

                    // create a move with a dummy savings value
                    Move m(Move::Ins,
                           fleet[vi][pi].getnid(), // nid1
                           -1,  // nid2
                           vi,  // vid1
                           vj,  // vid2
                           pi,  // pos1
                           pj,  // pos2
                           0.0);    // dummy savings

                    // we check two feasable conditions above but
                    // isNotFeasible tests if the move is possible
                    // or rejected by the lower level move methods
                    // TODO see if removing this changes the results
                    if (isNotFeasible(m)) continue;

                    // compute the savings and set it in the move
                    m.setsavings(getMoveSavings(m));

                    moves.push_back(m);
                    ++cnt;
                }
            }
        }
    }

    return cnt;
}


int OptSol::addRelatedMovesIntraSw(std::deque<Move>& moves, const Move& lastMove)  const {
    int cnt = 0;
    bool all = lastMove.getmtype() == Move::Invalid or not moves.size();
    int va = lastMove.getvid1();

    // generate moves based on the list of vid positions
    // and add them to the moves container
    // iterate through each vehicle (vi)
    for (int vi=0; vi<fleet.size(); vi++) {

        // we only need to regenerate moves for the vehicle in lastMove
        // unless all is true
        if (not all and not vi == va) continue;

        // iterate through the nodes in the path (pi, pj)
        // dont exchange the depot in position 0
        for (int pi=1; pi<fleet[vi].size(); pi++) {

            // dont try to move the dump
            if(fleet[vi][pi].isdump()) continue;

            // since we are swapping node, swap(i,j) == swap(j,i)
            // we only need to search forward
            for (int pj=pi+1; pj<fleet[vi].size(); pj++) {

                // dont try to move the dump
                if(fleet[vi][pj].isdump()) continue;

                // create a move with a dummy savings value
                Move m(Move::IntraSw,
                       fleet[vi][pi].getnid(), // nid1
                       fleet[vi][pj].getnid(), // nid2
                       vi,  // vid1
                       vi,  // vid2
                       pi,  // pos1
                       pj,  // pos2
                       0.0);    // dummy savings

                if (isNotFeasible(m)) continue;

                m.setsavings(getMoveSavings(m));

                moves.push_back(m);
                ++cnt;
            }
        }
    }

    return cnt;
}


int OptSol::addRelatedMovesInterSw(std::deque<Move>& moves, const Move& lastMove)  const {
    int cnt = 0;
    bool all = lastMove.getmtype() == Move::Invalid;
    int va = lastMove.getvid1();
    int vb = lastMove.getvid2();

    // generate moves based on the list of vid positions
    // and add them to the moves container

    // iterate through the vechicles (vi, vj)
    for (int vi=0; vi<fleet.size(); vi++) {
        for (int vj=0; vj<fleet.size(); vj++) {
            if (vi==vj) continue;
            // assume we have vehicles 0-10
            // and lastMove has vid1=va and vid2=vb
            // then we want to regenerate moves where
            //  vi      vj
            //------------
            //  va  -> 0-10
            //  vb  -> 0-10
            // 0-10 ->  va
            // 0-10 ->  vb
            // unless we need to generate all moves
            //
            if ( not all and
                 not ( (vi == va or vi == vb) or
                       (vj == va or vj == vb) ) ) continue;

            // iterate through the positions of each path (pi, pj)
            // dont exchange the depot in position 0
            for (int pi=1; pi<fleet[vi].size(); pi++) {
                // dont try to move the dump
                if(fleet[vi][pi].isdump()) continue;

                for (int pj=1; pj<fleet[vj].size(); pj++) {
                    // dont try to move the dump
                    if(fleet[vj][pj].isdump()) continue;

                    // create a move with a dummy savings value
                    Move m(Move::InterSw,
                           fleet[vi][pi].getnid(), // nid1
                           fleet[vj][pj].getnid(), // nid2
                           vi,  // vid1
                           vj,  // vid2
                           pi,  // pos1
                           pj,  // pos2
                           0.0);    // dummy savings
                    if (isNotFeasible(m)) continue;
                    double savings = getMoveSavings(m);
                    m.setsavings(savings);
                    moves.push_back(m);
                    ++cnt;
                }
            }
        }
    }

    return cnt;
}




// this should be dumb and fast as it is called a HUGE number of times
// The Ins move is defined as follows:
//      mtype = Move::Ins
//      vid1 - vehicle from
//      nid1 - node id in vehicle 1
//      pos1 - postion of nid1 in vid1
//      vid2 - destination vehicle
//      nid2 = -1 unused
//      pos2 - position to insert nid1 in vid2
//
// Algorithm
//   for every pickup node in every vehicle
//      create Move objects for moving that node to every position
//      in every other route if the Move would be feasable
//
void OptSol::getInsNeighborhood(std::deque<Move>& moves, const Move& lastMove)  const {
    Timer t0;
    int removed = clearRelatedMoves(moves, lastMove);
    int added = addRelatedMovesIns(moves, lastMove);

    // ---- ALL of the FOLLOWING is DEBUG and STATS collection ------
std::cout << "getInsNeighborhood: neighborhood updated: "
          << -removed << ", " << added
          << ", " << moves.size() << ", time: "
          << t0.duration();
lastMove.dump();

    STATS->addto("cum Ins removed", removed);
    STATS->addto("cum Ins added", added);

    std::vector<int> stats(fleet.size(), 0);
    for (std::deque<Move>::const_iterator it = moves.begin();
            it != moves.end(); ++it) {
        ++stats[it->getvid1()];
        ++stats[it->getvid2()];
    }
    std::cout << "=== neigborhood stats ===\n";
    for (int i=0; i<stats.size(); ++i)
        if (stats[i]) std::cout << "\tvpos["<<i<<"] = "<<stats[i]<<std::endl;
    std::cout << "=========================\n";
}


// this should be dumb and fast as it is called a HUGE number of times
// IntraSw move is defined as follows:
//      mtype = Move::IntraSw
//      vid1 - vehicle we are changing
//      nid1 - node id 1 that we are swapping
//      pos1 - position of nid1 in vid1
//      vid2 = -1 unused
//      nid2 - node id 2 that will get swapped with node id 1
//      pos2 - position of nid2 in vid1
//
// Algorithm
//  for every vehicle
//      for every pickup node
//          try to swap that node to every other position
//          within its original vehicle
//
void OptSol::getIntraSwNeighborhood(std::deque<Move>& moves, const Move& lastMove)  const {
    Timer t0;
    int removed = clearRelatedMoves(moves, lastMove);
    int added = addRelatedMovesIntraSw(moves, lastMove);

    // ---- ALL of the FOLLOWING is DEBUG and STATS collection ------
std::cout << "getIntraSwNeighborhood: neighborhood updated: "
          << -removed << ", " << added
          << ", " << moves.size() << ", time: "
          << t0.duration();
lastMove.dump();

    STATS->addto("cum IntraSw removed", removed);
    STATS->addto("cum IntraSw added", added);

    std::vector<int> stats(fleet.size(), 0);
    for (std::deque<Move>::const_iterator it = moves.begin();
            it != moves.end(); ++it) {
        ++stats[it->getvid1()];
        ++stats[it->getvid2()];
    }
    std::cout << "=== neigborhood stats ===\n";
    for (int i=0; i<stats.size(); ++i)
        if (stats[i]) std::cout << "\tvpos["<<i<<"] = "<<stats[i]<<std::endl;
    std::cout << "=========================\n";

}


// this should be dumb and fast as it is called a HUGE number of times
// The InterSw move is defined as follows:
//      mtype = Move::InterSw
//      vid1 - vehicle 1
//      nid1 - node id in vehicle 1
//      pos1 - postion of nid1 in vid1
//      vid2 - vehicle 2
//      nid2 = node id in vehicle 2
//      pos2 - position od nid2 in vid2
//
void OptSol::getInterSwNeighborhood(std::deque<Move>& moves, const Move& lastMove)  const {
    Timer t0;
    int removed = clearRelatedMoves(moves, lastMove);
    int added = addRelatedMovesInterSw(moves, lastMove);

    // ---- ALL of the FOLLOWING is DEBUG and STATS collection ------
std::cout << "getInsNeighborhood: neighborhood updated: "
          << -removed << ", " << added
          << ", " << moves.size() << ", time: "
          << t0.duration();
lastMove.dump();

    STATS->addto("cum InterSw removed", removed);
    STATS->addto("cum InterSw added", added);

    std::vector<int> stats(fleet.size(), 0);
    for (std::deque<Move>::const_iterator it = moves.begin();
            it != moves.end(); ++it) {
        ++stats[it->getvid1()];
        ++stats[it->getvid2()];
    }
    std::cout << "=== neigborhood stats ===\n";
    for (int i=0; i<stats.size(); ++i)
        if (stats[i]) std::cout << "\tvpos["<<i<<"] = "<<stats[i]<<std::endl;
    std::cout << "=========================\n";
}

bool OptSol::applyInsMove( const Move &move) {
        assert(move.getmtype() == Move::Ins);
        fleet[ move.getInsFromTruck() ].applyMoveINSerasePart(move.getnid1(), move.getpos1());
        fleet[ move.getInsToTruck() ].applyMoveINSinsertPart(datanodes[ move.getnid1() ], move.getpos2());
        assert( fleet[ move.getInsFromTruck() ].feasable() );
        assert( fleet[ move.getInsToTruck() ].feasable() );
        return (fleet[ move.getInsFromTruck() ].feasable() and  fleet[ move.getInsToTruck() ].feasable() );
}


// 2 vehicles involved
bool OptSol::applyInterSwMove( const Move &move) {
        assert(move.getmtype() == Move::InterSw);
        assert(not (move.getInterSwTruck1()==move.getInterSwTruck2()));
        assert(fleet[move.getInterSwTruck1()][ move.getpos1()].getnid()  == move.getnid1() );
        assert(fleet[move.getInterSwTruck2()][ move.getpos2()].getnid()  == move.getnid2() );

        fleet[move.getInterSwTruck1()].applyMoveInterSw( fleet[move.getInterSwTruck2()],  move.getpos1(),move.getpos2() )  ;

        assert( fleet[ move.getInterSwTruck1() ].feasable() );
        assert( fleet[ move.getInterSwTruck2() ].feasable() );
        return (fleet[ move.getInterSwTruck1() ].feasable() and  fleet[ move.getInterSwTruck2() ].feasable() );
}

//1 vehichle involved
bool OptSol::applyIntraSwMove( const Move &move) {
        assert(move.getmtype() == Move::IntraSw);
        assert(fleet[move.getIntraSwTruck()][ move.getpos1()].getnid()  == move.getnid1() );
        assert(fleet[move.getIntraSwTruck()][ move.getpos2()].getnid()  == move.getnid2() );

        fleet[move.getIntraSwTruck()].applyMoveIntraSw(  move.getpos1(),move.getpos2() )  ;

        assert( fleet[ move.getIntraSwTruck() ].feasable() );
        return (fleet[ move.getIntraSwTruck() ].feasable()) ;
}


#ifdef VICKY
////////////////////VIcky's part of the file
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



#endif
