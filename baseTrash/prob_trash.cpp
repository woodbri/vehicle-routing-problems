#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

//#include "order.h"
#include "prob_trash.h"

// Class functions

bool Prob_trash::checkIntegrity() const {
   bool flag=true;
   int nodesCant=datanodes.size();

   if (datanodes.empty()) {
        std::cout << "Nodes is empty\n";
        flag=false; }
   else std::cout << "# of Nodes:"<<nodesCant<<"\n";

   for (int i=1;i<nodesCant-1;i++) {
     flag= flag and datanodes[i].isvalid();
   }
}


// DUMPS ********************************************

void Prob_trash::nodesdump() {
    std::cout << "---- Nodes  --------------\n";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}


void Prob_trash::nodesdumpeval() {
    std::cout << "---- Nodes  Evaluation--------------\n";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dumpeval();
}


void Prob_trash::dump() {
    std::cout << "---- Problem -------------\n";
    nodesdump();
    std::cout << "INITIAL EVALUATION\n";
    nodesdumpeval();

}


void Prob_trash::dumpdataNodes() const {
    std::cout << "--------- Nodes ------------" << std::endl;
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}



void Prob_trash::dumpDepots() const {
    std::cout << "--------- Depots ------------" << std::endl;
    depots.dump("Depots");
    for (int i=0; i<depots.size(); i++)
        depots[i].dump();
}



void Prob_trash::dumpDumps() const {
    std::cout << "--------- Dumps ------------" << std::endl;
    dumps.dump("Dumps");
    for (int i=0; i<dumps.size(); i++)
        dumps[i].dump();
}


void Prob_trash::dumpPickups() const {
    std::cout << "--------- Pickups ------------" << std::endl;
    pickups.dump("pickups");
    for (int i=0; i<pickups.size(); i++)
        pickups[i].dump();
}


// PLOTS  ********************************************

void Prob_trash::plot(Plot<Trashnode> &graph) {
    for (int i=0; i<datanodes.size(); i++){
        if (datanodes[i].ispickup())  {
             graph.drawPoint(datanodes[i], 0x0000ff, 9, true);
        } else if (datanodes[i].isdump()) {
             graph.drawPoint(datanodes[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(datanodes[i], 0xff0000, 7, true);
        }
    }
};

Prob_trash::Prob_trash(const char *infile)
     {
std::cout << "---- char * Constructor --------------\n";
        std::string file = infile;
        loadProblem(file);
     } 

Prob_trash::Prob_trash(const std::string &infile) {
std::cout << "Prob_trash---- string Constructor --------------\n";
    loadProblem(infile);
}

/* depot must be the first node in list... rest can be anywhere*/

void Prob_trash::loadProblem(const std::string &infile)
{
    datafile=infile;
    Bucket nodes;
    Bucket intersection;
std::cout << "Prob_trash LoadProblem --------------"<< datafile<< "--------\n";


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

intersection.dump("intersection");
invalid.dump("invalid");


    nodes = pickups+otherlocs;

    for (int i=0;i<nodes.size();i++) {
        nodes[i].setnid(i);
        id = nodes[i].getid();
        if ( pickups.hasid( id ) ) pickups[ pickups.posFromId( id ) ].setnid(i);
        else if ( otherlocs.hasid( id ) ) otherlocs[ otherlocs.posFromId( id ) ].setnid(i);
    }

    assert( pickups.size() and otherlocs.size() );

    datanodes=nodes;


    twc.loadAndProcess_distance(datafile+".dmatrix-time.txt", datanodes,invalid);  
    Bucket dummy;
    dummy.setTravelTimes(twc.TravelTime());
    Tweval dummyNode;
    dummyNode.setTravelTimes(twc.TravelTime());
    assert( Tweval::TravelTime.size() );

    load_trucks(datafile+".vehicles.txt");
    assert(trucks.size() and depots.size() and dumps.size() and endings.size());
    

nodes.dump("nodes");
dumps.dump("dumps");
depots.dump("depots");
pickups.dump("pickups");
endings.dump("endings");
datanodes.dump("datanodes");
invalid.dump("invalid");

std::cout<<"TRUCKS\n";
for (int i=0;i<trucks.size();i++)
   trucks[i].tau();
std::cout<<"\n";
std::cout<<"INVALID TRUCKS\n";
for (int i=0;i<invalidTrucks.size();i++)
   invalidTrucks[i].tau();
std::cout<<"\n";
//twc.dump();
}

void Prob_trash::load_trucks(std::string infile) {
    assert (otherlocs.size());
    std::ifstream in( infile.c_str() );
    std::string line;
std::cout<<"Prob_trash:LoadTrucks"<<infile<<"\n";

    trucks.clear();
    while ( getline(in, line) ) {

        if (line[0] == '#') continue;
        Vehicle truck(line,otherlocs);  
        if (truck.isvalid()) {
            trucks.push_back(truck);
            depots.push_back(truck.getStartingSite());
            dumps.push_back(truck.getdumpSite());
            endings.push_back(truck.getEndingSite());
        }
        else { invalidTrucks.push_back(truck);
        }
    }
    in.close();
    
}

void Prob_trash::load_depots(std::string infile) { 
std::cout<<"Prob_trash:Load_depots"<<infile<<"\n";
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;

    depots.clear();
    while ( getline(in, line) ) {
        cnt++;
        if (line[0] == '#') continue;

        Trashnode node(line);  
        if ( not node.isvalid() or not node.isdepot()) {
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
           invalid.push_back(node);
        } else {
           depots.push_back(node); 
        }
    }
    in.close();
}

void Prob_trash::load_otherlocs(std::string infile) { 
std::cout<<"Prob_trash:Load_otherlocs"<<infile<<"\n";
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;

    otherlocs.clear();
    while ( getline(in, line) ) {
        cnt++;

        if (line[0] == '#') continue;

        Trashnode node(line);  
        if ( not node.isvalid() ) {
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
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
        if ( not node.isvalid() or not node.isdump()) {
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
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
    while ( getline(in, line) ) {
        cnt++;
        if (line[0] == '#') continue;
        Trashnode node(line);  

        if ( not node.isvalid() ) {
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
           invalid.push_back(node);
        } else {
          node.setType(2);
          pickups.push_back(node);
        }
    }

    in.close();
}

