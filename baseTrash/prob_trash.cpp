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
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "logger.h"
#include "prob_trash.h"

// Class functions

bool Prob_trash::checkIntegrity() const {
   bool flag=true;
   int nodesCant=datanodes.size();

   if (datanodes.empty()) {
        DLOG(INFO) << "Nodes is empty";
        flag=false; }
   else DLOG(INFO) << "# of Nodes:" << nodesCant;

   for (int i=1;i<nodesCant-1;i++) {
     flag= flag and datanodes[i].isValid();
   }
}


// DUMPS ********************************************

void Prob_trash::nodesdump() {
    DLOG(INFO) << "---- Nodes  --------------";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}


void Prob_trash::nodesdumpeval() {
    DLOG(INFO) << "---- Nodes  Evaluation--------------";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dumpeval();
}


void Prob_trash::dump() {
    DLOG(INFO) << "---- Problem -------------";
    nodesdump();
    DLOG(INFO) << "INITIAL EVALUATION";
    nodesdumpeval();

}


void Prob_trash::dumpdataNodes() const {
    DLOG(INFO) << "--------- Nodes ------------";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}



void Prob_trash::dumpDepots() const {
    DLOG(INFO) << "--------- Depots ------------";
    depots.dump("Depots");
    for (int i=0; i<depots.size(); i++)
        depots[i].dump();
}



void Prob_trash::dumpDumps() const {
    DLOG(INFO) << "--------- Dumps ------------";
    dumps.dump("Dumps");
    for (int i=0; i<dumps.size(); i++)
        dumps[i].dump();
}


void Prob_trash::dumpPickups() const {
    DLOG(INFO) << "--------- Pickups ------------";
    pickups.dump("pickups");
    for (int i=0; i<pickups.size(); i++)
        pickups[i].dump();
}


// PLOTS  ********************************************

void Prob_trash::plot(Plot<Trashnode> &graph) {
    for (int i=0; i<datanodes.size(); i++){
        if (datanodes[i].isPickup())  {
             graph.drawPoint(datanodes[i], 0x0000ff, 9, true);
        } else if (datanodes[i].isDump()) {
             graph.drawPoint(datanodes[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(datanodes[i], 0xff0000, 7, true);
        }
    }
};

Prob_trash::Prob_trash(const char *infile)
     {
#ifdef LOG
    DLOG(INFO) << "---- char * Constructor --------------";
#endif
        std::string file = infile;
        loadProblem(file);
     } 

Prob_trash::Prob_trash(const std::string &infile) {
#ifdef LOG
    DLOG(INFO) << "Prob_trash---- string Constructor --------------";
#endif
    loadProblem(infile);
}

/* depot must be the first node in list... rest can be anywhere*/

void Prob_trash::loadProblem(const std::string &infile)
{
    datafile=infile;
    Bucket nodes;
    Bucket intersection;
#ifdef LOG
    DLOG(INFO) << "Prob_trash LoadProblem --------------" << datafile
               << "--------";
#endif


    // read the nodes
    int cnt=0;
    int nid=0;
    int id=0;

    load_pickups(datafile+".containers.txt");
    load_otherlocs(datafile+".otherlocs.txt");

    intersection = otherlocs * pickups;
    invalid += intersection;
    pickups -= intersection;
    nodes -= intersection;

#ifndef LOG
    invalid.dump("invalid");
#endif


    nodes = pickups+otherlocs;
    nodes.push_back(C);
    for (int i=0;i<nodes.size();i++) {
        nodes[i].setnid(i);
        id = nodes[i].getid();
        if ( pickups.hasId( id ) )
            pickups[ pickups.posFromId( id ) ].setnid(i);
        else if ( otherlocs.hasId( id ) )
            otherlocs[ otherlocs.posFromId( id ) ].setnid(i);
    };
    C=nodes.back();
    assert( pickups.size() );
    assert( otherlocs.size() );

    datanodes=nodes;



    twc->loadAndProcess_distance(datafile+".dmatrix-time.txt", datanodes,invalid);  
    twc->setHints(dumps);
    twc->setHints(nodes);
    twc->setHints(depots);
    twc->setHints(pickups);
    twc->setHints(endings);

    twc->settCC(C,pickups);
//    Bucket dummy;
//    dummy.setTravelTimes(twc->TravelTime());
//    C.setTravelTimes(twc->TravelTime());

//    assert( Tweval::TravelTime.size() );

//    buildStreets(pickups);

    load_trucks(datafile+".vehicles.txt");
    assert(trucks.size() and depots.size() and dumps.size() and endings.size());
    for (int i=0;i<trucks.size();i++) {
	trucks[i].setInitialValues(C,pickups);
    }
    
#ifdef LOG
    C.dump();
    nodes.dump("nodes");
    dumps.dump("dumps");
    depots.dump("depots");
    pickups.dump("pickups");
    endings.dump("endings");
    datanodes.dump("datanodes");
    invalid.dump("invalid");
    DLOG(INFO) << "TRUCKS";
    for (int i=0;i<trucks.size();i++)
       trucks[i].tau();
    DLOG(INFO) << "INVALID TRUCKS";
    for (int i=0;i<invalidTrucks.size();i++)
       invalidTrucks[i].tau();
    twc->dump();
#endif
}

void Prob_trash::buildStreets( Bucket &unassigned, Bucket &assigned) {

#ifdef TESTED
    DLOG(INFO) << "Build Streets";
#endif
    Bucket assign;
    Trashnode node;
    if ( not unassigned.size()) return;
    node= unassigned[0];
    Street  street( node );
    unassigned.pop_front();
    assigned.push_back( node );
    street.e_insert(unassigned,assign);
    street.dumpid();
    assigned+=assign;
    streets.push_back(street);
    buildStreets(unassigned,assigned);
}
    


void Prob_trash::buildStreets(const Bucket &nodes) {
#ifdef TESTED
    DLOG(INFO) << "Build Streets";
#endif
    Bucket assigned;
    Bucket unassigned=nodes;
    Street street;
    buildStreets (unassigned, assigned);
}


void Prob_trash::load_trucks(std::string infile) {
    assert (otherlocs.size());
    std::ifstream in( infile.c_str() );
    std::string line;
#ifdef TESTED
    DLOG(INFO) << "Prob_trash:LoadTrucks" << infile;
#endif

    trucks.clear();
    while ( getline(in, line) ) {

        if (line[0] == '#') continue;
        Vehicle truck(line,otherlocs);  
        if (truck.isvalid()) {
            trucks.push_back(truck);
            depots.push_back(truck.getStartingSite());
            dumps.push_back(truck.getDumpSite());
            endings.push_back(truck.getEndingSite());
        }
        else { invalidTrucks.push_back(truck);
        }
    }
    in.close();
    
}

void Prob_trash::load_depots(std::string infile) { 
    DLOG(INFO) << "Prob_trash:Load_depots" << infile;
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;

    depots.clear();
    while ( getline(in, line) ) {
        cnt++;
        if (line[0] == '#') continue;

        Trashnode node(line);  
        if ( not node.isValid() or not node.isDepot()) {
           DLOG(INFO) << "ERROR: line: " << cnt << ": " << line;
           invalid.push_back(node);
        } else {
           depots.push_back(node); 
        }
    }
    in.close();
}

void Prob_trash::load_otherlocs(std::string infile) { 
    DLOG(INFO) << "Prob_trash:Load_otherlocs" << infile;
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;

    otherlocs.clear();
    while ( getline(in, line) ) {
        cnt++;

        if (line[0] == '#') continue;

        Trashnode node(line);  
        if ( not node.isValid() ) {
           DLOG(INFO) << "ERROR: line: " << cnt << ": " << line;
           invalid.push_back(node);
        } else {
           otherlocs.push_back(node);  
        }
    }
    in.close();
}


void Prob_trash::load_dumps(std::string infile) { //1 dump problem
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;
    dumps.clear();
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node(line);  
        if ( not node.isValid() or not node.isDump()) {
           DLOG(INFO) << "ERROR: line: " << cnt << ": " << line;
           invalid.push_back(node);
        } else {
           dumps.push_back(node);  
        }
    }
    in.close();
}

void Prob_trash::load_pickups(std::string infile) {
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;
    pickups.clear();
    double st,op,cl,dm,x,y;
    st=op=cl=dm=x=y=0;
    
    while ( getline(in, line) ) {
        cnt++;
        if (line[0] == '#') continue;
        Trashnode node(line);  
        node.setType(2);
        if ( not node.isValid() ) {
#ifdef TESTED
           DLOG(INFO) << "ERROR: line: " << cnt << ": " << line;
#endif
           invalid.push_back(node);
        } else {
          pickups.push_back(node);
	  st+=node.getServiceTime();
	  op+=node.opens();
	  cl+=node.closes();
	  dm+=node.getDemand();
	  x+=node.getx();
	  y+=node.gety();
        }
    }

    in.close();
	st=st/pickups.size();
	op=op/pickups.size();
	cl=cl/pickups.size();
	dm=dm/pickups.size();
	x=x/pickups.size();
	y=y/pickups.size();
	C.set(-1,-1,x,y,dm,op,cl,st);
}

