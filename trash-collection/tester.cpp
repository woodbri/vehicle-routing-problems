
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "vec2d.h"
#include <stdio.h>

#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "twpath.h"
#include "trashproblem.h"

void TestDistanceFromLineSegmentToPoint( double segmentX1, double segmentY1, double segmentX2, double segmentY2, double pX, double pY ) {
    double qX;
    double qY;
    double d = distanceFromLineSegmentToPoint( segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, &qX, &qY );
    printf( "line segment = ( ( %f, %f ), ( %f, %f ) ), p = ( %f, %f ), distance = %f, q = ( %f, %f )\n",
            segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, d, qX, qY );
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
    std::cout << "Usage: tester in.txt\n";
}

int main(int argc, char **argv) {

    std::string font = "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf";

    if (argc < 2) {
        Usage();
        return 1;
    }

    std::string infile = argv[1];

    try {

//#define TEST2OPT
//#define TESTSWAP
#ifdef TESTSWAP

        infile = "p52.txt";

        TrashProblem tp;
        tp.loadproblem( infile );
        tp.dumpdataNodes();

        do {
            int sol[] = {0,7,8,11,14,17,20,4,0,-1,
                         1,5,9,12,15,18,21,4,1,-1,
                         2,6,10,13,16,19,22,4,2,-1};
            std::vector<int> solution(sol, sol+sizeof(sol)/sizeof(int));

            if (!tp.buildFleetFromSolution(solution)) {
                std::cout << "Problem failed to load!" << std::endl;
                return 1;
            }
            tp.dump();

            Vehicle v0 = tp.getVehicle(0);
            Vehicle v1 = tp.getVehicle(1);
            Vehicle v2 = tp.getVehicle(2);

            std::cout << "\nv0.swap3(v1, v2, 1, 1, 1)" << std::endl;
            std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
            v0.dumppath();
            v1.dumppath();
            v2.dumppath();
            v0.swap3(v1, v2, 1, 1, 1);
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
                return 1;
            }
            tp.dump();

            Vehicle v0 = tp.getVehicle(0);
            Vehicle v1 = tp.getVehicle(1);
            Vehicle v2 = tp.getVehicle(2);

            std::cout << "\nv0.exchange3(v1, v2, 2, 1, 1, 1)" << std::endl;
            std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
            v0.dumppath();
            v1.dumppath();
            v2.dumppath();
            v0.exchange3(v1, v2, 2, 1, 1, 1);
            std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
            v0.dumppath();
            v1.dumppath();
            v2.dumppath();
        } while (false);

#else
#ifdef TEST2OPT


        infile = "p50.txt";

        TrashProblem tp;
        tp.loadproblem( infile );
        tp.dumpdataNodes();

        do {
            int sol[] = { 0,25,24,23,32,22,14,15,7,8,16,9,17,5,0,-1,
                          1,10,11,12,13,21,40,41,31,20,30,29,19,18,5,1,-1,
                          2,34,33,42,50,51,44,43,45,46,27,28,26,35,6,2,-1,
                          3,52,53,54,47,55,56,49,48,39,38,37,36,6,3,-1 };
            std::vector<int> solution(sol, sol+sizeof(sol)/sizeof(int));

            if (!tp.buildFleetFromSolution(solution)) {
                std::cout << "Problem failed to load!" << std::endl;
                return 1;
            }
            tp.dump();

            Vehicle v1 = tp.getVehicle(1);
            Vehicle v2 = tp.getVehicle(2);

            std::cout << "\nv1.pathTwoOpt()" << std::endl;
            std::cout << "oldcost: " << v1.getcost() << "\n";
            v1.dumppath();
            v1.pathTwoOpt();
            std::cout << "newcost: " << v1.getcost() << "\n";
            v1.dumppath();
        } while (false);

#else

        // ----------------------------------------------------------------

        TrashProblem tp;

        tp.loadproblem( infile );

        tp.dumpdataNodes();
        //tp.dumpDmatrix();
        tp.dump();

/*
        int a = tp.findNearestNodeTo(2, PICKUP, 0);
        std::cout << "tp.findNearestNodeTo(2, PICKUP) = " << a << std::endl;
        a = tp.findNearestNodeTo(2, DUMP, 0);
        std::cout << "tp.findNearestNodeTo(2, DUMP) = " << a << std::endl;
        a = tp.findNearestNodeTo(2, DUMP|PICKUP, 0);
        std::cout << "tp.findNearestNodeTo(2, DUMP|PICKUP) = " << a << std::endl;
        a = tp.findNearestNodeTo(2, PICKUP|LIMITDEMAND, 90);
        std::cout << "tp.findNearestNodeTo(2, PICKUP|LIMITDEMAND,90) = " << a << std::endl;

        std::cout << "\n----------- dumbConstruction -----------------------\n";
        tp.dumbConstruction();
        tp.dump();
        tp.plot("p0.png", "dumbConstruction", font);

        std::cout << "\n---------------------------------------------------\n";

*/

/*
        std::cout << "\n----------- nearestNeighbor -----------------------\n";
        tp.nearestNeighbor();
        tp.dump();
        tp.plot("p1.png", "nearestNeighbor", font);

        tp.dump();

        std::cout << "\n---------------------------------------------------\n";
        std::cout << "\n---------------------------------------------------\n";
        std::cout << "\n---------------------------------------------------\n";

        //tp.loadproblem( infile );
        //tp.dump();

*/
        std::cout << "\n----------- assignmentSweep -----------------------\n";
        tp.assignmentSweep();
        tp.dump();
        tp.plot("p2.png", "assignmentSweep", font);


        Vehicle v = tp.getVehicle(0);
        Twpath<Trashnode> p = v.getvpath();

/*
        std::cout << "\nv.pathOptMoveNodes()" << std::endl;
        v.dumppath();
        v.pathOptMoveNodes();
        v.dumppath();

        v = tp.getVehicle(0);
        std::cout << "\nv.pathOptExchangeNodes()" << std::endl;
        v.dumppath();
        v.pathOptExchangeNodes();
        v.dumppath();

        v = tp.getVehicle(0);
        std::cout << "\nv.pathOptInvertSequence()" << std::endl;
        v.dumppath();
        v.pathOptInvertSequence();
        v.dumppath();

*/
        int sz = p.size();
        std::cout << "\np.move(2,"<<sz-2<<","<<sz<<",v.getmaxcapacity())" << std::endl;
        p.dump();
        p.e_move(2,p.size()-2,p.size(),v.getmaxcapacity());
        p.dump();

/*
        p = v.getvpath();
        std::cout << "\np.move(2,4,10,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.move(2,4,10,v.getmaxcapacity());
        p.dump();

        p = v.getvpath();
        std::cout << "\np.move(7,9,5,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.move(7,9,5,v.getmaxcapacity());
        p.dump();

        p = v.getvpath();
        std::cout << "\np.move(7,9,3,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.move(7,9,3,v.getmaxcapacity());
        p.dump();

        p = v.getvpath();
        std::cout << "\np.movereverse(2,4,7,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.movereverse(2,4,7,v.getmaxcapacity());
        p.dump();

        p = v.getvpath();
        std::cout << "\np.movereverse(2,4,10,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.movereverse(2,4,10,v.getmaxcapacity());
        p.dump();

        p = v.getvpath();
        std::cout << "\np.movereverse(7,9,5,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.movereverse(7,9,5,v.getmaxcapacity());
        p.dump();

        p = v.getvpath();
        std::cout << "\np.movereverse(7,9,3,v.getmaxcapacity())" << std::endl;
        p.dump();
        p.movereverse(7,9,3,v.getmaxcapacity());
        p.dump();

*/

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
/*
    do {
        Vehicle v1 = tp.getVehicle(0);
        Vehicle v2 = tp.getVehicle(1);

        std::cout << "\nv1.swap(v2, 12, 14)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.swap(v2, 12, 14);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        Vehicle v1 = tp.getVehicle(0);
        Vehicle v2 = tp.getVehicle(1);

        std::cout << "\nv1.exchangeTails(v2, 12, 14)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.exchangeTails(v2, 12, 14);
        std::cout << "newcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
    } while (false);

    do {
        Vehicle v0 = tp.getVehicle(1);
        Vehicle v1 = tp.getVehicle(2);
        Vehicle v2 = tp.getVehicle(3);

        // validated move by commenting out improvement test in vehicle.cpp
        std::cout << "\nv1.exchangeTails(v2, 3, 1)" << std::endl;
        std::cout << "oldcost: " << v1.getcost() + v2.getcost() << "\n";
        v1.dumppath();
        v2.dumppath();
        v1.exchangeTails(v2, 3, 1);
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

*/
/*
        std::cout << "\n----------- assignmentSweep3 -----------------------\n";
        tp.setRatio(0.9);
        tp.assignmentSweep3();
        tp.dump();

        tp.plot("p3.png", "assignmentSweep3", font);

        std::cout << "\n----------- doing 3-opt -----------------------\n";
        tp.opt_3opt();
        tp.dumpSummary();
        tp.plot("p5.png", "assignmentSweep3 - after 3opt", font);
*/
/*

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

        std::cout << "\n----------- doing pathOptimize ---------------------\n";
        tp.optimize();
        tp.dump();
        tp.plot("p7.png", "assignmentSweep2 - after optimize", font);
*/
#endif
#endif
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
