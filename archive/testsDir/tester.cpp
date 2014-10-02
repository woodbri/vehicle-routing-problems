
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "node.h"
#include "twnode.h"
#include "trashnode.h"
#include "dpnode.h"
#include "twpath.h"


void Usage() {
    std::cout << "Usage: tester in.txt\n";
}

int main(int argc, char **argv) {

    if (argc < 2) {
        Usage();
        return 1;
    }

    char * infile = argv[1];
    //try {
        Node mydepot(0,0,0);
	Twpath<Node> p;
        p.setdepot(mydepot);
        p.push_back(mydepot);
        Node n;
        for (int i=10;i<20;i++){
          n.set(i,i,i);
          p.push_back(n);
        }
        p.dump();
    //}
    try {

        Node n;
        std::cout << "Empty Node validity: " << n.isvalid() << std::endl;
        n.set(1, 10, 20);
        n.dump();
        std::cout << "Node validity: " << n.isvalid() << std::endl;

        Node n2(2, 11,21);
        n2.dump();
        std::cout << "n[1].isSamePos(n[2]): " << n.isSamePos(n2) << std::endl;
        std::cout << "n[1].isSamePos(n[2],1.5): " << n.isSamePos(n2, 1.5) << std::endl;
        std::cout << "n[1].distance(n[2]): " << n.distance(n2) << std::endl;

        Twnode tw;
        tw.set(2, 20, 30, 40, 120, 600, 5);
        tw.dump();
        std::cout << "tw.opens(): " << tw.opens() << std::endl;
        std::cout << "tw.closes(): " << tw.closes() << std::endl;
        std::cout << "tw.getdemand(): " << tw.getdemand() << std::endl;
        std::cout << "tw.getservicetime(): " << tw.getservicetime() << std::endl;
        std::cout << "tw.windowlength(): " << tw.windowlength() << std::endl;
        std::cout << "tw.checkintegrity(): " << tw.checkintegrity() << std::endl;
        std::cout << "tw.hasdemand(): " << tw.hasdemand() << std::endl;
        std::cout << "tw.hassupply(): " << tw.hassupply() << std::endl;
        std::cout << "tw.earlyarrival(110): " << tw.earlyarrival(110) << std::endl;
        std::cout << "tw.earlyarrival(120): " << tw.earlyarrival(120) << std::endl;
        std::cout << "tw.earlyarrival(620): " << tw.earlyarrival(620) << std::endl;
        std::cout << "tw.latearrival(110): " << tw.latearrival(110) << std::endl;
        std::cout << "tw.latearrival(600): " << tw.latearrival(600) << std::endl;
        std::cout << "tw.latearrival(620): " << tw.latearrival(620) << std::endl;

        Trashnode tn;
        std::cout << "tn.hasdemand(): " << tn.hasdemand() << std::endl;
        std::cout << "tn.hassupply(): " << tn.hassupply() << std::endl;
        std::cout << "tn.hasnogoods(): " << tn.hasnogoods() << std::endl;
        tn.setvalues(3, 21, 32, 40, 120, 600, 5, 2);
        tn.dump();
        std::cout << "tn.isvalid(): " << tn.isvalid() << std::endl;
        std::cout << "tn.opens(): " << tn.opens() << std::endl;
        std::cout << "tn.closes(): " << tn.closes() << std::endl;
        std::cout << "tn.getdemand(): " << tn.getdemand() << std::endl;
        std::cout << "tn.getservicetime(): " << tn.getservicetime() << std::endl;
        std::cout << "tn.windowlength(): " << tn.windowlength() << std::endl;
        std::cout << "tn.checkintegrity(): " << tn.checkintegrity() << std::endl;
        std::cout << "tn.hasdemand(): " << tn.hasdemand() << std::endl;
        std::cout << "tn.hassupply(): " << tn.hassupply() << std::endl;
        std::cout << "tn.isdepot(): " << tn.isdepot() << std::endl;
        std::cout << "tn.isdump(): " << tn.isdump() << std::endl;
        std::cout << "tn.ispickupnode(): " << tn.ispickupnode() << std::endl;
        std::cout << "tn.earlyarrival(110): " << tn.earlyarrival(110) << std::endl;
        std::cout << "tn.earlyarrival(120): " << tn.earlyarrival(120) << std::endl;
        std::cout << "tn.earlyarrival(620): " << tn.earlyarrival(620) << std::endl;
        std::cout << "tn.latearrival(110): " << tn.latearrival(110) << std::endl;
        std::cout << "tn.latearrival(600): " << tn.latearrival(600) << std::endl;
        std::cout << "tn.latearrival(620): " << tn.latearrival(620) << std::endl;

        Trashnode depot(1, 0, 0, 0, 0, 24*60, 0, 0);
        Trashnode dumpsite(2, 10, 10, 0, 0, 24*60, 30, 1);
        Twpath <Trashnode> route;
        route.setdepot(depot);
        route.setdumpsite(dumpsite);
        route.dump();
        route.push_back(tn);
        tn = Trashnode(4, 15, 35, 10, 0, 24*60, 15, 2);
        route.push_front(tn);
        route.dump();

        Dpnode dtn;
        std::cout << "dtn.hasdemand(): " << dtn.hasdemand() << std::endl;
        std::cout << "dtn.hassupply(): " << dtn.hassupply() << std::endl;
        std::cout << "dtn.hasnogoods(): " << dtn.hasnogoods() << std::endl;
        dtn.set(3, 21, 32, 40, 120, 600, 5);
        dtn.dump();
        std::cout << "dtn.isvalid(): " << dtn.isvalid() << std::endl;
        std::cout << "dtn.opens(): " << dtn.opens() << std::endl;
        std::cout << "dtn.closes(): " << dtn.closes() << std::endl;
        std::cout << "dtn.getdemand(): " << dtn.getdemand() << std::endl;
        std::cout << "dtn.getservicetime(): " << dtn.getservicetime() << std::endl;
        std::cout << "dtn.windowlength(): " << dtn.windowlength() << std::endl;
        std::cout << "dtn.checkintegrity(): " << dtn.checkintegrity() << std::endl;
        std::cout << "dtn.hasdemand(): " << dtn.hasdemand() << std::endl;
        std::cout << "dtn.hassupply(): " << dtn.hassupply() << std::endl;
        std::cout << "dtn.isdepot(): " << dtn.isdepot() << std::endl;
        std::cout << "dtn.ispickup(): " << dtn.ispickup() << std::endl;
        std::cout << "dtn.isdelivery(): " << dtn.isdelivery() << std::endl;
        std::cout << "dtn.earlyarrival(110): " << dtn.earlyarrival(110) << std::endl;
        std::cout << "dtn.earlyarrival(120): " << dtn.earlyarrival(120) << std::endl;
        std::cout << "dtn.earlyarrival(620): " << dtn.earlyarrival(620) << std::endl;
        std::cout << "dtn.latearrival(110): " << dtn.latearrival(110) << std::endl;
        std::cout << "dtn.latearrival(600): " << dtn.latearrival(600) << std::endl;
        std::cout << "dtn.latearrival(620): " << dtn.latearrival(620) << std::endl;

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
