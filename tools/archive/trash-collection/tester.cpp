
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <stdio.h>

#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "trashproblem.h"

void TestDistanceFromLineSegmentToPoint( double segmentX1, double segmentY1, double segmentX2, double segmentY2, double pX, double pY ) {
    Node s1(1, segmentX1, segmentY1);
    Node s2(2, segmentX2, segmentY2);
    Node p1(3, pX, pY);
    double d = p1.distanceToSegment( s1, s2 );
    printf( "line segment = ( ( %f, %f ), ( %f, %f ) ), p = ( %f, %f ), distance = %f\n",
            segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, d );
}

void TestDistanceFromLineSegmentToPoint() {
    TestDistanceFromLineSegmentToPoint( 0, 0, 1, 1, 1, 0 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, 5, 4 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, 30, 15 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, -30, 15 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 10, 0, 5, 1 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 0, 10, 1, 5 );
}

void Usage() {
    std::cout << "Usage: tester <option> in.txt\n";
    std::cout << "  <option>:\n";
    std::cout << "      trash        - run the trash collection problem\n";
    std::cout << "      swap         - test swap nodes between routes\n";
    std::cout << "      2opt         - test 2-opt optimization of a route\n";
    std::cout << "      movesbetween - test moves between multiple routes\n";
    std::cout << "      moveswithin  - test moves within a route via e_*\n";
    std::cout << "      moveswithinnew  - test moves within a route via vehicle\n";
    std::cout << "      dumb         - test dump construction\n";
    std::cout << "      p10a         - test p10a route\n";
    std::cout << "      nearestnode  - test findNearestNode\n";
    std::cout << "      optmoves     - test various optimization moves\n";
}

void test_swap(std::string infile);
void test_2opt(std::string infile);
void test_trash(std::string infile);
void test_movesbetween(std::string infile);
void test_moveswithin(std::string infile);
void test_moveswithinnew(std::string infile);
void test_findNearestNodeTo(std::string infile);
void test_dumbcons(std::string infile);
void test_p10a(std::string infile);
void test_optmoves(std::string infile);

static std::string font = "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf";

int main(int argc, char **argv) {

    if (argc < 3) {
        Usage();
        return 1;
    }

    std::string option = argv[1];
    std::string infile = argv[2];
    

    try {

        if      (option == "trash")        test_trash(infile);
        else if (option == "swap")         test_swap("p52.txt");
        else if (option == "2opt")         test_2opt("p50.txt");
        else if (option == "movesbetween") test_movesbetween("p50.txt");
        else if (option == "moveswithin")  test_moveswithin("p50.txt");
        else if (option == "moveswithinnew")  test_moveswithinnew("p50.txt");
        else if (option == "dumb")         test_dumbcons("p10.txt");
        else if (option == "p10a")         test_p10a("p10a.txt");
        else if (option == "nearestnode")  test_findNearestNodeTo("p50.txt");
        else if (option == "optmoves")     test_optmoves("p50.txt");
        else {
            TestDistanceFromLineSegmentToPoint();
            Usage();
            return 1;
        }

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}


//-------------------------------------------------------------
// do swap and exchange tests
//-------------------------------------------------------------
void test_swap(std::string infile) {

    TrashProblem tp;
    tp.loadproblem( infile );
    //tp.dumpdataNodes();

    do {
        int sol[] = {0,7,8,11,14,17,20,4,0,-1,
                     1,5,9,12,15,18,21,4,1,-1,
                     2,6,10,13,16,19,22,4,2,-1};
        std::vector<int> solution(sol, sol+sizeof(sol)/sizeof(int));

        if (!tp.buildFleetFromSolution(solution)) {
            std::cout << "Problem failed to load!" << std::endl;
        }
        //tp.dumpFleet();

        Vehicle v0 = tp.getVehicle(0);
        Vehicle v1 = tp.getVehicle(1);
        Vehicle v2 = tp.getVehicle(2);

        std::cout << "\nv0.swap3(v1, v2, 1, 1, 1)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v0.dumppath();
        v1.dumppath();
        v2.dumppath();
        v0.swap3(v1, v2, 1, 1, 1, false);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v0.dumppath();
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        //tp.loadproblem( infile );

        int sol2[] = {0,7,10,11,14,17,20,4,0,-1,
                      1,5,8,12,15,18,21,4,1,-1,
                      2,6,9,13,16,19,22,4,2,-1};
        std::vector<int> solution2(sol2, sol2+sizeof(sol2)/sizeof(int));

        if (!tp.buildFleetFromSolution(solution2)) {
            std::cout << "Problem failed to load!" << std::endl;
        }
        //tp.dump();

        Vehicle v0 = tp.getVehicle(0);
        Vehicle v1 = tp.getVehicle(1);
        Vehicle v2 = tp.getVehicle(2);

        std::cout << "\nv0.exchange3(v1, v2, 2, 1, 1, 1)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v0.dumppath();
        v1.dumppath();
        v2.dumppath();
        v0.exchange3(v1, v2, 2, 1, 1, 1, false);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v0.dumppath();
        v1.dumppath();
        v2.dumppath();
    } while (false);

}



//----------------------------------------------------------------
// do 2opt tests
//----------------------------------------------------------------
void test_2opt(std::string infile) {

    TrashProblem tp;
    tp.loadproblem( infile );
    //tp.dumpdataNodes();

    do {
        int sol[] = { 0,25,24,23,32,22,14,15,7,8,16,9,17,5,0,-1,
                      1,10,11,12,13,21,40,41,31,20,30,29,19,18,5,1,-1,
                      2,34,33,42,50,51,44,43,45,46,27,28,26,35,6,2,-1,
                      3,52,53,54,47,55,56,49,48,39,38,37,36,6,3,-1 };
        std::vector<int> solution(sol, sol+sizeof(sol)/sizeof(int));

        if (!tp.buildFleetFromSolution(solution)) {
            std::cout << "Problem failed to load!" << std::endl;
        }
        //tp.dump();

        Vehicle v1 = tp.getVehicle(1);
        Vehicle v2 = tp.getVehicle(2);

        std::cout << "\nv1.pathTwoOpt()" << std::endl;
        std::cout << "oldcost: " << v1.getcost() << "\n";
        v1.dumppath();
        v1.pathTwoOpt();
        std::cout << "newcost: " << v1.getcost() << "\n";
        v1.dumppath();
    } while (false);

}


//---------------------------------------------------------------
// run trash collection problem
//---------------------------------------------------------------
void test_p10a(std::string infile) {
    TrashProblem tp;
    tp.loadproblem( infile );
    tp.dumpdataNodes();


    do {
        int sol[] = { 0,7,8,4,3,2,6,5,9,10,11,12,13,1,0,-1 };
        std::vector<int> solution(sol, sol+sizeof(sol)/sizeof(int));

        if (!tp.buildFleetFromSolution(solution)) {
            std::cout << "Problem failed to load!" << std::endl;
        }

        //Vehicle v = tp.getVehicle(0);
        //v.dump();

        tp.dump();

    } while(false);

}


//---------------------------------------------------------------
// run trash collection problem
//---------------------------------------------------------------
void test_trash(std::string infile) {

        TrashProblem tp;

        tp.loadproblem( infile );

        tp.dumpdataNodes();
        //tp.dumpDmatrix();
        tp.dump();

        std::cout << "\n----------- assignmentSweep3 -----------------------\n";
        tp.setRatio(0.9);
        tp.assignmentSweep3();
        tp.dump();

        tp.plot("p3.png", "assignmentSweep3", font);

/*
        std::cout << "\n----------- doing 3-opt -----------------------\n";
        tp.opt_3opt();
        tp.dumpSummary();
        tp.plot("p5.png", "assignmentSweep3 - after 3opt", font);

*/
        std::cout << "\n----------- doing 2-opt -----------------------\n";
        tp.opt_2opt();
        tp.dumpSummary();
        tp.plot("p4.png", "assignmentSweep3 - after 2opt", font);

        std::cout << "\n----------- doing or-opt -----------------------\n";
        tp.opt_or_opt();
        tp.dumpSummary();
        tp.plot("p6.png", "assignmentSweep3 - after or-opt", font);

        std::cout << "\n----------- doing 2-opt again -----------------\n";
        tp.opt_2opt();
        tp.dumpSummary();
        tp.plot("p7.png", "assignmentSweep3 - after 2opt again", font);

/*
        std::cout << "\n----------- doing pathOptimize ---------------------\n";
        tp.optimize();
        tp.dump();
        tp.plot("p7.png", "assignmentSweep2 - after optimize", font);
*/

}

//----------------------------------------------------------------------
// test moves
//----------------------------------------------------------------------

/*
tests for:
bool swap(Vehicle& v2, const int& i1, const int& i2);
bool swap3(Vehicle& v2, Vehicle& v3, const int& i1, const int& i2, const int& i3);
bool exchangeSeq(Vehicle& v2, const int& i1, const int& j1, const int& i2, const int& j2);
bool exchangeTails(Vehicle& v2, const int& i1, const int& i2);
bool exchange3(Vehicle& v2, Vehicle& v3, const int& cnt, const int& i1, const int& i2, const int& i3);
bool relocate(Vehicle& v2, const int& i1, const int& i2);
bool relocateBest(Vehicle& v2, const int& i1);
*/

void test_movesbetween(std::string infile) {

    TrashProblem tp;

    tp.loadproblem( infile );
    tp.assignmentSweep();

    //tp.dumpdataNodes();
    //tp.dumpDmatrix();
    //tp.dump();

    do {
        Vehicle v1 = tp.getVehicle(0);
        Vehicle v2 = tp.getVehicle(1);

        std::cout << "\nv1.swap2(v2, 12, 13, false)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.swap2(v2, 12, 13, false);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        Vehicle v1 = tp.getVehicle(0);
        Vehicle v2 = tp.getVehicle(1);

        std::cout << "\nv1.exchangeTails(v2, 12, 13, false)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.exchangeTails(v2, 12, 13, false);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        Vehicle v0 = tp.getVehicle(1);
        Vehicle v1 = tp.getVehicle(2);
        Vehicle v2 = tp.getVehicle(3);

        // validated move by commenting out improvement test in vehicle.cpp
        std::cout << "\nv1.exchangeTails(v2, 3, 1, false)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.exchangeTails(v2, 3, 1, false);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();

        std::cout << "\nv0.relocateBest(v1, 9)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v0.getcost() << "\n";
        v0.dumppath();
        v1.dumppath();
        v0.relocateBest(v1, 9);
        std::cout << "newcost: " << v1.getcost() + v0.getcost() << "\n";
        v0.dumppath();
        v1.dumppath();
    } while (false);

    do {
        Vehicle v1 = tp.getVehicle(0);
        Vehicle v2 = tp.getVehicle(1);

        std::cout << "\nv1.swap2(v2, 12, 13, true)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.swap2(v2, 12, 13, true);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        Vehicle v1 = tp.getVehicle(0);
        Vehicle v2 = tp.getVehicle(1);

        std::cout << "\nv1.exchangeTails(v2, 12, 13, true)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.exchangeTails(v2, 12, 13, true);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        Vehicle v0 = tp.getVehicle(1);
        Vehicle v1 = tp.getVehicle(2);
        Vehicle v2 = tp.getVehicle(3);

        // validated move by commenting out improvement test in vehicle.cpp
        std::cout << "\nv1.exchangeTails(v2, 3, 1, true)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.exchangeTails(v2, 3, 1, true);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();

    } while (false);

}

void test_findNearestNodeTo(std::string infile) {
    TrashProblem tp;

    tp.loadproblem( infile );

    tp.dumpdataNodes();
    //tp.dump();

    int a = tp.findNearestNodeTo(2, PICKUP, 0);
    std::cout << "tp.findNearestNodeTo(2, PICKUP) = " << a << std::endl;
    a = tp.findNearestNodeTo(2, DUMP, 0);
    std::cout << "tp.findNearestNodeTo(2, DUMP) = " << a << std::endl;
    a = tp.findNearestNodeTo(2, DUMP|PICKUP, 0);
    std::cout << "tp.findNearestNodeTo(2, DUMP|PICKUP) = " << a << std::endl;
    a = tp.findNearestNodeTo(2, PICKUP|LIMITDEMAND, 90);
    std::cout << "tp.findNearestNodeTo(2, PICKUP|LIMITDEMAND,90) = " << a << std::endl;

}


//----------------------------------------------------------------------
// test construction
//----------------------------------------------------------------------

void test_dumbcons(std::string infile) {
    TrashProblem tp;

    tp.loadproblem( infile );

    tp.dumpdataNodes();
    //tp.dump();

    std::cout << "\n----------- dumbConstruction -----------------------\n";
    tp.dumbConstruction();
    tp.dump();
    tp.plot("p0.png", "dumbConstruction", font);

    std::cout << "\n---------------------------------------------------\n";

}

void test_optmoves(std::string infile) {
    double oldcost, newcost;

    TrashProblem tp;
    tp.loadproblem( infile );

    //tp.dumpdataNodes();
    //tp.dumpDmatrix();
    //tp.dump();

    //std::cout << "\n----------- assignmentSweep -----------------------\n";
    tp.assignmentSweep();
    //tp.dump();
    //tp.plot("p2.png", "assignmentSweep", font);

    Vehicle v = tp.getVehicle(0);

    std::cout << "\nv.pathOptMoveNodes()" << std::endl;
    oldcost = v.getcost();
    std::cout << "oldcost: " << oldcost << " : ";
    v.dumppath();
    v.pathOptMoveNodes();
    newcost = v.getcost();
    std::cout << "newcost: " << newcost << " : ";
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.pathOptExchangeNodes()" << std::endl;
    oldcost = v.getcost();
    std::cout << "oldcost: " << oldcost << " : ";
    v.dumppath();
    v.pathOptExchangeNodes();
    newcost = v.getcost();
    std::cout << "newcost: " << newcost << " : ";
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.pathOptInvertSequence()" << std::endl;
    oldcost = v.getcost();
    std::cout << "oldcost: " << oldcost << " : ";
    v.dumppath();
    v.pathOptInvertSequence();
    newcost = v.getcost();
    std::cout << "newcost: " << newcost << " : ";
    v.dumppath();

}


void test_moveswithin(std::string infile) {

    TrashProblem tp;

    tp.loadproblem( infile );

    //tp.dumpdataNodes();
    //tp.dumpDmatrix();
    //tp.dump();

    //std::cout << "\n----------- assignmentSweep -----------------------\n";
    tp.assignmentSweep();
    //tp.dump();
    //tp.plot("p2.png", "assignmentSweep", font);

    Vehicle v = tp.getVehicle(0);
    Twpath<Trashnode> p = v.getvpath();
    int sz = p.size();
    E_Ret ret;

    p = v.getvpath();
    std::cout << "\np.e_move(2,"<<sz-2<<","<<sz<<",v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_move(2,p.size()-2,p.size(),v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_move(2,4,10,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_move(2,4,10,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_move(7,9,5,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_move(7,9,5,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_move(7,9,3,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_move(7,9,3,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_movereverse(2,4,7,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_movereverse(2,4,7,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_movereverse(2,4,10,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_movereverse(2,4,10,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_movereverse(7,9,5,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_movereverse(7,9,5,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

    p = v.getvpath();
    std::cout << "\np.e_movereverse(7,9,3,v.getmaxcapacity())" << std::endl;
    p.dump();
    ret = p.e_movereverse(7,9,3,v.getmaxcapacity());
    if (ret == OK) p.dump();
    else std::cout << "RETURNED: " << (ret==INVALID)?"INVALID\n":"NO_CHANGE\n";

}



void test_moveswithinnew(std::string infile) {

    TrashProblem tp;

    tp.loadproblem( infile );

    //tp.dumpdataNodes();
    //tp.dumpDmatrix();
    //tp.dump();

    //std::cout << "\n----------- assignmentSweep -----------------------\n";
    tp.assignmentSweep();
    //tp.dump();
    //tp.plot("p2.png", "assignmentSweep", font);

    Vehicle v = tp.getVehicle(0);
    int sz = v.size();

    std::cout << "\nv.moverange(2,"<<sz-2<<","<<sz<<")" << std::endl;
    v.dumppath();
    v.moverange(2,v.size()-2,v.size());
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.moverange(2,4,10)" << std::endl;
    v.dumppath();
    v.moverange(2,4,10);
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.moverange(7,9,5)" << std::endl;
    v.dumppath();
    v.moverange(7,9,5);
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.moverange(7,9,3)" << std::endl;
    v.dumppath();
    v.moverange(7,9,3);
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.movereverse(2,4,7)" << std::endl;
    v.dumppath();
    v.movereverse(2,4,7);
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.movereverse(2,4,10)" << std::endl;
    v.dumppath();
    v.movereverse(2,4,10);
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.movereverse(7,9,5)" << std::endl;
    v.dumppath();
    v.movereverse(7,9,5);
    v.dumppath();

    v = tp.getVehicle(0);
    std::cout << "\nv.movereverse(7,9,3)" << std::endl;
    v.dumppath();
    v.movereverse(7,9,3);
    v.dumppath();

}

