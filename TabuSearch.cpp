
#include "TabuSearch.h"
#include "Plot.h"
#include <cstdio>

inline void swap(int& a, int& b) {
    int tmp = a;
    a = b;
    b = tmp;
}

inline void dumpvec(const std::vector<int>& v) {
    for (int i=0; i<v.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << v[i];
    }
    std::cout << std::endl;
}


Solution TabuSearch::solve() {

    T.clear();      // clear the Tabu list
    tabuLength = std::max(30, (int)S.P.O.size());
std::cout << "tabuLength: " << tabuLength << std::endl;

    iter = 0;   // init the iteration counter
    int maxIter = 500;

    // get the average time window length
    double atwl = S.P.atwl;

    while (iter++ < maxIter) {

std::cout << "---------- TabuSearch::solve: iter: " << iter << std::endl;

        int nwri = S.P.N.size()/14;
        double ardl = S.getAverageRouteDurationLength();
        if (atwl/ardl > 0.25) 
            nwri = S.P.N.size()/10;
        else if (atwl/ardl < 0.25)
            nwri = S.P.N.size()/25;

        if (doSPI()) {
            for (int i=0; i<nwri; i++) {
                if (! doWRI()) break;
            }
        }
        else if (doSBR()) {
            for (int i=0; i<nwri; i++) {
            if (! doWRI()) break;
            }
        }
        else
            break;

        cleanTabuList();

        Plot plot(S);
        char file[100];
        sprintf(file, "out/vrpdptw-%04d.png", iter);
        plot.out(file, true, 800, 800, file);

    }

//std::cout << "TabuSearch::solve: Best Solution is: " << std::endl;
//Best.dump();

std::cout << "TabuSearch::solve: TabuList" << std::endl;
for (int i=0; i<T.size(); i++)
    T[i].dump();

    return S;
}

//SPI - Single Pair Insertion
    // for each route
        // get the route
        // identify the orders in this route
        // for each order in that route
            // try to move it to another route
                // can't move to ourself                

                // if valid predecessor node insertion
                // then place successor node if all location following it

                // if we are eliminating a route
                // then the new solution must be feasible otherwise 
                // it is ok to have in infeasible route
bool TabuSearch::doSPI() {
//std::cout << "Enter TabuSearch::doSPI(): " << std::endl;;
    // initialize bestMove
    bestMove.moveType = -1;
    bestMove.savings = -std::numeric_limits<double>::max();
    bestMove.savings = 0.0;

    // oid==0 is the depot
    for ( int oid=1; oid<S.P.O.size(); oid++) {
        if (S.mapOtoR[oid] == -1) {
            std::cout << "ERROR: S.mapOtoR[oid] == -1 for oid: "
                      << oid << std::endl;
            continue;
        }

        // copy the route this order is in and get the cost with the order
        Route r1(S.R[S.mapOtoR[oid]]);
        double r1oldc = r1.getCost();

        // remove the order can get the cost minus the order
        r1.removeOrder(oid);
        double r1newc = r1.getCost();

        //get number of orders in this route
        int nord = r1.orders.size() / 2;

        for ( int rid=0; rid<S.R.size(); rid++) {
            // can't move order to self
            if (S.mapOtoR[oid] == rid) continue;

            // copy the route to work with it
            Route r2(S.R[rid]);
            double r2oldc = r2.getCost();

            // try to place it in the new route
            // if it is eliminating a route then the move MUST be valid
            //bool ok = r2.insertOrder(oid, nord==0);
            bool ok = r2.insertOrder(oid, true);
            if (!ok) continue;

            // calculate the savings
            double r2newc = r2.getCost();

            // savings is the sum of costs before the change 
            // minus the sum of the costs after the change
            double savings = (r1oldc + r2oldc) - (r1newc + r2newc);

/*
std::cout << "doSPI: oid: " << oid << ", rid1: " << r1.rid << ", rid2: " << r2.rid << std::endl;
std::cout << "       r1oldc: " << r1oldc << ", r2oldc: " << r2oldc << " = " << r1oldc + r2oldc << std::endl;
std::cout << "       r1newc: " << r1newc << ", r2newc: " << r2newc << " = " << r1newc + r2newc << std::endl;
std::cout << "       savings: " << (r1oldc + r2oldc) -(r1newc + r2newc) << std::endl;
std::cout << "       r2: "; r2.dump();
*/

            Move m;
            m.moveType = 1;
            m.oid1 = oid;
            m.oid2 = -1;
            m.rid1 = S.mapOtoR[oid];
            m.rid2 = rid;
            m.savings = savings;
            int k = 0;
            for (int j=0; j<r2.orders.size(); j++) {
                if (r2.orders[j] == oid) {
                    if (k==0) {
                        m.ppos1 = j;
                        k++;
                    }
                    else {
                        m.spos1 = j;
                        break;
                    }
                }
            }

//m.dump();

            // if this move is better the the bestMove then save it
            if ( m.savings > bestMove.savings
                 &&
                 ( SCost - m.savings < BestCost   // aspirational
                   ||
                   ! isMoveTabu(m)           // or not tabu
                 ) ) {
                bestMove = m;
/*
std::cout << "   SCost: " << SCost << ", savings: " << savings << ", BestCost: " << BestCost << std::endl;
std::cout << "   (SCost - savings < BestCost): " << (SCost - savings < BestCost) << ", isMoveTabu(m): " << isMoveTabu(m) << std::endl;
std::cout << "   bestMove.moveType: " << bestMove.moveType << ", bestMove.savings: " << bestMove.savings << std::endl;

std::cout << "New best move assigned: " << std::endl;
m.dump();
*/
            }
        }
    }

    // if we found no valid moves, return false
    if (bestMove.moveType == -1) return false;

std::cout << "SPI: BestMove: SCost: " << SCost << ": ";
bestMove.dump();

    // otherwise update the current solution and apply the best move
    applyMove(bestMove);

//std::cout << "Applied bestMove" << std::endl;
//S.dump();

    return true;
}


bool TabuSearch::doSBR() {
std::cout << "Enter TabuSearch::doSBR(): " << std::endl;;
    // initialize bestMove
    bestMove.moveType = -1;
    bestMove.savings = -std::numeric_limits<double>::max();

    // for each order
    // oid==0 is the depot
    for ( int oid1=1; oid1<S.P.O.size(); oid1++) {
        int currentRoute = S.mapOtoR[oid1];
        // swap it for another order not in the current route
        for ( int oid2=1; oid2<S.P.O.size(); oid2++) {
            if (oid1 == oid2) continue;
            if (currentRoute == S.mapOtoR[oid2]) continue;
            Route r1(S.R[currentRoute]);
            double r1oldc = r1.getCost();
            Route r2(S.R[S.mapOtoR[oid2]]);
            double r2oldc = r2.getCost();

            Move m;
            m.moveType = 2;
            m.oid1 = oid1;
            m.oid2 = oid2;
            m.rid1 = r1.rid;
            m.rid2 = r2.rid;

            std::vector<int> r1p(r1.path);
            std::vector<int> r2p(r2.path);

            for (int i=0; i<r1p.size(); i++) {
                if (r1p[i] == S.P.O[oid1].pid) {
                    r1p[i] = S.P.O[oid2].pid;
                    m.ppos1 = i;
                }
                else if (r1p[i] == S.P.O[oid1].did) {
                    r1p[i] = S.P.O[oid2].did;
                    m.spos1 = i;
                    break;
                }
            }

            for (int i=0; i<r2p.size(); i++) {
                if (r2p[i] == S.P.O[oid2].pid) {
                    r2p[i] = S.P.O[oid1].pid;
                    m.ppos2 = i;
                }
                else if (r2p[i] == S.P.O[oid2].did) {
                    r2p[i] = S.P.O[oid1].did;
                    m.spos2 = i;
                    break;
                }
            }
/*
if (m.ppos1 == -1 || m.spos1 == -1 || m.ppos2 == -1 || m.spos2 == -1 ) {
    std::cout << "####### ERROR: SBR: r1.rid: " << r1.rid << ", r2.rid: " << r2.rid << std:: endl;
    std::cout << "    old r1: "; dumpvec(S.R[S.mapOtoR[oid1]].path);
    std::cout << "    old r2: "; dumpvec(S.R[S.mapOtoR[oid2]].path);
    std::cout << "    new r1: "; dumpvec(r1p);
    std::cout << "    new r2: "; dumpvec(r2p);
    std::cout << "    move: ";
    m.dump();
    std::cout << "##################" << std:: endl;
}
*/
            double r1newc = r1.testPath(r1p);
            double r2newc = r2.testPath(r2p);

            m.savings = (r1oldc + r2oldc) - (r1newc + r2newc);

//m.dump();

            // if this move is better the the bestMove then save it
            if ( m.savings > bestMove.savings
                 &&
                 ( SCost - m.savings < BestCost   // aspirational
                   ||
                   ! isMoveTabu(m)           // or not tabu
                 ) ) {
                bestMove = m;
            }
        }
    }

    // if a best move was found return true else return false
    if (bestMove.moveType == -1)
        return false;

std::cout << "SBR: BestMove: SCost: " << SCost << ": ";
bestMove.dump();

    // otherwise update the current solution and apply the best move
    applyMove(bestMove);

//std::cout << "Applied bestMove" << std::endl;
//S.dump();

    return true;
}


bool TabuSearch::doWRI() {
    std::vector<int> np;

//std::cout << "Enter TabuSearch::doWRI(): " << std::endl;;
    // initialize bestMove
    bestMove.moveType = -1;
    bestMove.savings  =  0;

    // for each route
    for (int rid=0; rid<S.R.size(); rid++) {
        // for each node move it forward and backwards
        //     looking for a lower cost position
        Route& r(S.R[rid]);
        double roldc = r.getCost();

        for (int i=0; i<r.path.size(); i++) {
            // move it forward
            np = r.path;
            for (int j=i-1; j>=0; j--) {
                Move m;
                // we cant move a successor before its predecessor
                if (r.orders[i] == r.orders[j]) break;
                // walk the node backwards in each iteration
                swap(np[j], np[j+1]);
                double rnewc = r.testPath(np);
//std::cout << "WRI(" << rid << "," << i << "," << j << "): roldc: " << roldc << ", rnewc: " << rnewc << ", savings: " << roldc - rnewc << std::endl;
                if (roldc - rnewc > bestMove.savings) {
                    m.moveType = 3;
                    m.rid1 = rid;
                    m.oid1 = r.orders[i];
                    m.oid2 = r.orders[j];
                    m.ppos1 = i;
                    m.ppos2 = j;
                    m.savings = roldc - rnewc;
                }

                if (m.moveType == -1) continue;

                // if this move is better the the bestMove then save it
                if ( m.savings > bestMove.savings
                     &&
                     ( SCost - m.savings < BestCost   // aspirational
                       ||
                       ! isMoveTabu(m)           // or not tabu
                     ) ) {
                    bestMove = m;
                }
            }

            // move it backward
            np = r.path;
            for (int j=i+1; j<r.path.size(); j++) {
                Move m;
                // we can move a predecessor node beyond it successor
                if (r.orders[i] == r.orders[j]) break;
                // walk the node forward in each iteration
                swap(np[j-1], np[j]);
                double rnewc = r.testPath(np);
                if (roldc - rnewc > bestMove.savings) {
                    m.moveType = 3;
                    m.rid1 = rid;
                    m.oid1 = r.orders[i];
                    m.oid2 = r.orders[j];
                    m.ppos1 = i;
                    m.ppos2 = j;
                    m.savings = roldc - rnewc;
                }

                if (m.moveType == -1) continue;

                // if this move is better the the bestMove then save it
                if ( m.savings > bestMove.savings
                     &&
                     ( SCost - m.savings < BestCost   // aspirational
                       ||
                       ! isMoveTabu(m)           // or not tabu
                     ) ) {
                    bestMove = m;
                }
            }
        }
    }

    // if a best move was found apply it and return true else return false
    if (bestMove.moveType == -1 || bestMove.savings <= 0)
        return false;

std::cout << "WRI: BestMove: SCost: " << SCost << ": ";
bestMove.dump();

    // otherwise update the current solution and apply the best move
    applyMove(bestMove);

//std::cout << "Applied bestMove" << std::endl;
//S.dump();

    return true;
}


double TabuSearch::getAverageRouteDurationLength() {
    int n = 0;
    int D = 0;
    for (int i=0; i<S.R.size(); i++) {
        if (S.R[i].orders.size() == 0) continue;
        if (S.R[i].updated) S.R[i].update();
        D += S.R[i].D;
        n++;
    }
    return D/n;
}


void TabuSearch::applyMove(Move& m) {

//std::cout << "TabuSearch::applyMove ----------------------" << std::endl;

//std::cout << "  move: ";
//m.dump();
/*
std::cout << "  bef SCost: " << SCost << std::endl;
std::cout << "  bef Route[" << m.rid1 << "](" << S.R[m.rid1].getCost()  << "): ";
if (m.rid1 != -1) S.R[m.rid1].dump(); else std::cout << std::endl;
std::cout << "  bef Route[" << m.rid2 << "](" << S.R[m.rid2].getCost()  << "): ";
if (m.rid2 != -1) S.R[m.rid2].dump(); else std::cout << std::endl;
*/
/*
if (m.oid1 == 42 || m.oid2 == 42) {
  std::cout << "******************** BEFORE\n";
  std::cout << "move: "; m.dump();
  if (m.rid1 != -1) {
    std::cout << "rid1: "; S.R[m.rid1].dump();
  }
  if (m.rid2 != -1) {
    std::cout << "rid2: "; S.R[m.rid2].dump();
  }
  std::cout << "***************************\n";
}
*/

    std::vector<int>::iterator it;
    int o, n, ppos2;
    switch (m.moveType) {
        case 1: // SPI move
            S.R[m.rid1].removeOrder(m.oid1);
            // insert the order in this route
            it = S.R[m.rid2].path.begin();
            S.R[m.rid2].path.insert(it+m.ppos1, S.P.N[S.P.O[m.oid1].pid].nid);
            it = S.R[m.rid2].path.begin();
            S.R[m.rid2].path.insert(it+m.spos1, S.P.N[S.P.O[m.oid1].did].nid);
            it = S.R[m.rid2].orders.begin();
            S.R[m.rid2].orders.insert(it+m.ppos1, m.oid1);
            it = S.R[m.rid2].orders.begin();
            S.R[m.rid2].orders.insert(it+m.spos1, m.oid1);
            S.R[m.rid2].updated = true;
            // update the mappind vector
            S.mapOtoR[m.oid1] = m.rid2;
            break;
        case 2: // SBR move
            // update the paths
            S.R[m.rid1].path[m.ppos1] = S.P.O[m.oid2].pid;
            S.R[m.rid1].path[m.spos1] = S.P.O[m.oid2].did;
            S.R[m.rid2].path[m.ppos2] = S.P.O[m.oid1].pid;
            S.R[m.rid2].path[m.spos2] = S.P.O[m.oid1].did;
            // update the orders
            S.R[m.rid1].orders[m.ppos1] = m.oid2;
            S.R[m.rid1].orders[m.spos1] = m.oid2;
            S.R[m.rid2].orders[m.ppos2] = m.oid1;
            S.R[m.rid2].orders[m.spos2] = m.oid1;
            // update the mapping of orders to routes
            S.mapOtoR[m.oid1] = m.rid2;
            S.mapOtoR[m.oid2] = m.rid1;
            // mark the routes as updated
            S.R[m.rid1].updated = true;
            S.R[m.rid2].updated = true;
            break;
        case 3: // WRI move
            n = S.R[m.rid1].path[m.ppos1];
            o = S.R[m.rid1].orders[m.ppos1];
            // because we delete ppos1 first
            // it will shift ppos2 if it is after it
            ppos2 = m.ppos2;
            if (m.ppos1 < m.ppos2) ppos2--;

            // update the path
            it = S.R[m.rid1].path.begin();
            S.R[m.rid1].path.erase(it+m.ppos1);
            it = S.R[m.rid1].path.begin();
            S.R[m.rid1].path.insert(it+ppos2, n);

            // update the orders vector
            it = S.R[m.rid1].orders.begin();
            S.R[m.rid1].orders.erase(it+m.ppos1);
            it = S.R[m.rid1].orders.begin();
            S.R[m.rid1].orders.insert(it+ppos2, o);

            S.R[m.rid1].updated = true;
            break;
        default:
            std::cout << "ERROR: TabuSearch::applyMove asked to apply a moveType: " << m.moveType << std::endl;
            return;
    }

    // update solution cost
    S.computeCosts();
    SCost = S.getCost();
/*
std::cout << "  aft SCost: " << SCost << std::endl;
std::cout << "  aft Route[" << m.rid1 << "](" << S.R[m.rid1].getCost()  << "): ";
if (m.rid1 != -1) S.R[m.rid1].dump(); else std::cout << std::endl;
std::cout << "  aft Route[" << m.rid2 << "](" << S.R[m.rid2].getCost()  << "): ";
if (m.rid2 != -1) S.R[m.rid2].dump(); else std::cout << std::endl;
*/
/*
if (m.oid1 == 42 || m.oid2 == 42) {
  std::cout << "******************** AFTER\n";
  std::cout << "move: "; m.dump();
  if (m.rid1 != -1) {
    std::cout << "rid1: "; S.R[m.rid1].dump();
  }
  if (m.rid2 != -1) {
    std::cout << "rid2: "; S.R[m.rid2].dump();
  }
  std::cout << "***************************\n";
}
*/

//std::cout << "TabuSearch::applyMove: SCost < BestCost : " << SCost << " < " << BestCost << std::endl;

    // if this is the Best Solution save it
    if (SCost < BestCost) {
//std::cout << "############# Best move updated!" << std::endl;
        Best = S;
        BestCost = SCost;
    }

    // Add move to the Tabu list
    addMoveTabu(m);
}

void TabuSearch::addMoveTabu(Move& m) {
    Tabu tm;

    tm.expires = iter + tabuLength;
    tm.checked = 0;
    tm.aspirational = 0;

    switch (m.moveType) {
        case 1:
            // moke the move tabu
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid2;
            tm.topos = m.ppos1;
            tm.move  = m.moveType;
            addTabu(tm);
            // prevent the move back also
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid1;
            tm.topos = -1;
            tm.move  = m.moveType;
            addTabu(tm);
            break;
        case 2:
            // make the moves tabu
            tm.node  = S.P.O[m.oid2].pid;
            tm.torid = m.rid1;
            tm.topos = m.ppos1;
            tm.move  = m.moveType;
            addTabu(tm);
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid2;
            tm.topos = m.ppos2;
            tm.move  = m.moveType;
            addTabu(tm);
            // make the moves back tabu
            tm.node  = S.P.O[m.oid2].pid;
            tm.torid = m.rid2;
            tm.topos = -1;
            tm.move  = m.moveType;
            addTabu(tm);
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid1;
            tm.topos = -1;
            tm.move  = m.moveType;
            addTabu(tm);
            break;
        case 3:
            /*
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid1;
            tm.topos = m.ppos2;
            addTabu(tm);
            */
            break;
        default:
            break;
    }
}


void TabuSearch::addTabu(Tabu &tm) {
    std::vector<Tabu>::iterator it;

    // search the Tabu list for this move
    it = std::find( T.begin(), T.end(), tm );

    if ( it == T.end() ) {
        // move is not already on the Tabu list so add it
        T.insert(T.begin(), tm);
        if (debugTabu) {
            std::cout << "TABU: move added at (" << iter << "): ";
            tm.dump();
        }
    }
    else {
        // it is already on the Tabu list 
        // so we must be making an aspiration move
        // so update the expires and checked counter
        it->expires = iter + tabuLength;
        it->checked++;
        it->aspirational++;
        if (debugTabu) {
            std::cout << "TABU: aspirational update at (" << iter << "): ";
            it->dump();
        }
    }
}


bool TabuSearch::isMoveTabu(Move& m) {
    Tabu tm;
    bool ret = false;

    tm.expires = iter + tabuLength;
    tm.checked = 0;
    tm.aspirational = 0;

    switch (m.moveType) {
        case 1:
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid2;
            tm.topos = m.ppos1;
            tm.move  = m.moveType;
            ret = ret || isTabu(tm);
            break;
        case 2:
            tm.node  = S.P.O[m.oid2].pid;
            tm.torid = m.rid1;
            tm.topos = m.ppos1;
            tm.move  = m.moveType;
            ret = ret || isTabu(tm);
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid2;
            tm.topos = m.ppos2;
            tm.move  = m.moveType;
            ret = ret || isTabu(tm);
            break;
        case 3:
            /*
            tm.node  = S.P.O[m.oid1].pid;
            tm.torid = m.rid1;
            tm.topos = m.ppos2;
            addTabu(tm);
            */
            break;
        default:
            break;
    }
    return ret;
}

bool TabuSearch::isTabu(Tabu& tm) {
    std::vector<Tabu>::iterator it;
    it = std::find( T.begin(), T.end(), tm );
    if ( it == T.end() )
        return false;
    if ( it->expires < iter ) {
        if (debugTabu) {
            std::cout << "TABU: removed expired at (" << iter << "): ";
            it->dump();
        }
        T.erase(it);
        return false;
    }
    else {
        it->checked++;
    }
    return true;
}


void TabuSearch::cleanTabuList() {
    while (T.back().expires < iter) {
        if (debugTabu) {
            std::cout << "TABU: cleaned expired at (" << iter << "): ";
            T.back().dump();
        }
        T.pop_back();
    }
}



