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

Prob_trash::Prob_trash(char *infile)
     {
std::cout << "---- Constructor --------------\n";
         loadProblem(infile);
     } 


/* depot must be the first node in list... rest can be anywhere*/
void Prob_trash::loadProblem(char *infile)
{
    datafile=std::string(infile);
std::cout << "---- Load --------------";
std::cout << datafile<< " ---- Load --------------\n";


    // read the nodes
    int cnt=0;
    int nid=0;
    load_dumps(datafile+".dumps.txt",nid);
    load_depots(datafile+".depots.txt",nid);
    load_pickups(datafile+".containers.txt",nid);
    load_trucks(datafile+".vehicles.txt");
    load_matrix(datafile+".dmatrix-time.txt");
    
//    twc.setNodes(datanodes);
//twc.dump();
}

void Prob_trash::load_trucks(std::string infile) { //1 dump problem
    assert (depots.size());
    assert (dumps.size());
    std::ifstream in( infile.c_str() );
    std::string line;
std::cout<<"Loading vehicles FILE"<<infile<<"\n";

    trucks.clear();
    int offset=dumps.size()+depots.size()-1;
    int cnt=0;
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;
        Vehicle truck(line,depots,dumps,offset);  //create truck from line on file
truck.tau();
        if (truck.isvalid()) trucks.push_back(truck);
        else invalidTrucks.push_back(truck);
    }
    in.close();
for (int i=0;i<trucks.size();i++)
   trucks[i].dumpeval();
    
}

void Prob_trash::load_depots(std::string infile, int &nid) { //1 dump problem
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;
std::cout<<"Loading depots FILE"<<infile<<"\n";

    depots.clear();
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node(line);  //create node from line on file
        node.setnid(nid);
node.dump();
        if ( not node.isvalid() or not node.isdepot()) {
           node.setnid(node.getid());
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
           invalid.push_back(node);
        } else {
           node.setnid(nid);
           datanodes.push_back(node);
           depots.push_back(node);  //just in case we need to select the closest dump, for now only one is there
        }
        nid++;
    }
datanodes.dump();
depots.dump();
    in.close();
}

void Prob_trash::load_dumps(std::string infile, int &nid) { //1 dump problem
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;
std::cout<<"Loading dumps FILE"<<infile<<"\n";
    dumps.clear();
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node(line);  //create node from line on file
        node.setnid(nid);
node.dump();
        if ( not node.isvalid() or not node.isdump()) {
           node.setnid(node.getid());
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
           invalid.push_back(node);
        } else {
           node.setnid(nid);
           datanodes.push_back(node);
           dumps.push_back(node);  //just in case we need to select the closest dump, for now only one is there
           nid++;
        }
    }
    in.close();
datanodes.dump();
dumps.dump();
}

void Prob_trash::load_pickups(std::string infile, int &nid) {
    std::ifstream in( infile.c_str() );
    std::string line;
    int cnt = 0;
std::cout<<"Loading pickups FILE"<<infile<<"\n";
    pickups.clear();
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;
        Trashnode node(line);  //create node from line on file
        node.setnid(nid);
node.dump();

        if ( not node.isvalid() or not node.ispickup()) {
           std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
           node.setnid(node.getid());
           invalid.push_back(node);
        } else {
           node.setnid(nid);
           datanodes.push_back(node);
           pickups.push_back(node);  //just in case we need to select the closest dump, for now only one is there
           nid++;
        }
    }
    in.close();
datanodes.dump();
pickups.dump();
}

void Prob_trash::load_matrix(std::string infile ) {
    std::ifstream in( infile.c_str() );
    std::string line;
    std::deque<int> fromId;
    std::deque<int> toId;
    std::deque<double> timeDist;
    int from,to;
    double dist;
    int cnt = 0;
std::cout<<"Loading matrix FILE"<<infile<<"\n";
    pickups.clear();
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;
        std::istringstream buffer( line );
        buffer >> from;
        buffer >> to;
        buffer >> dist;
        fromId.push_back(from);
        toId.push_back(to);
        timeDist.push_back(dist);
    }
    in.close();
for (int i=0; i<fromId.size();i++) 
    std::cout<<"("<<fromId[i]<<" . "<<toId[i]<<")="<<timeDist[i]<<"\n";
std::cout<<"processing matrix data  goes here \n";

}



