#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

//#include "order.h"
#include "prob_trash.h"

// Class functions

bool Prob_pd::checkIntegrity() const {
   bool flag=true;
   int nodesCant=datanodes.size();
//   int ordersCant=ordersList.size();

   if (datanodes.empty()) {
        std::cout << "Nodes is empty\n";
        flag=false; }
   else std::cout << "# of Nodes:"<<nodesCant<<"\n";

/*
   if (ordersList.empty()) {
        std::cout << "Orders is empty\n";
        flag=false;}
   else std::cout << "# of Orders:"<<ordersCant<<"\n";
   if (ordersCant != (nodesCant-1)/2) {
        std::cout << "Expected "<<(nodesCant-1)/2<<" Orders. Found "<<ordersCant<<" Orders\n";
        flag=false;}
   else std::cout << "Found expected # of Orders\n";
*/
   for (int i=1;i<nodesCant-1;i++) {
     flag= flag and datanodes[i].isvalid();
   }
}


void Prob_pd::nodesdump() {
    std::cout << "---- Nodes  --------------\n";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dump();
}


void Prob_pd::nodesdumpeval() {
    std::cout << "---- Nodes  Evaluation--------------\n";
    for (int i=0; i<datanodes.size(); i++)
        datanodes[i].dumpeval();
}


void Prob_pd::dump() {
    std::cout << "---- Problem -------------\n";
    nodesdump();
    std::cout << "INITIAL EVALUATION\n";
    nodesdumpeval();

}

void Prob_pd::plot(Plot<Trashnode> &graph) {
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

Prob_pd::Prob_pd(char *infile)
     {
std::cout << "---- Constructor --------------\n";
         loadProblem(infile);
     } 


/* depot must be the first node in list... rest can be anywhere*/
void Prob_pd::loadProblem(char *infile)
{
    datafile=std::string(infile);
std::cout << "---- Load --------------";
std::cout << datafile<< " ---- Load --------------\n";


    // read the nodes
    int cnt=0;
    int nid=0;
    load_dumps(datafile+".dumps.txt",cnt,nid);
    load_depots(datafile+".depots.txt",cnt,nid);
//    load_pickups(in,cnt,nid);
//    load_trucks(in,depots,dumps);
    
//    twc.setNodes(datanodes);
//twc.dump();
}

void Prob_pd::load_trucks(std::string infile, const Bucket &depots, const Bucket &dumps) { //1 dump problem
    assert (depots.size());
    assert (dumps.size());
    std::ifstream in( infile.c_str() );
    std::string line;
    trucks.clear();
    int offset=dumps.size()+depots.size()-1;
    int cnt=0;
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;
        if (line[0] == '$') break;
        Vehicle truck(line,depots,dumps,offset);  //create truck from line on file
        if (truck.isvalid()) trucks.push_back(truck);
        else invalidTrucks.push_back(truck);
    }
    in.close();
}

void Prob_pd::load_depots(std::string infile,int &foo, int &nid) { //1 dump problem
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

void Prob_pd::load_dumps(std::string infile,int &foo, int &nid) { //1 dump problem
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

void Prob_pd::load_pickups(std::string infile,int &cnt, int &nid) {
    std::ifstream in( infile.c_str() );
    std::string line;
    pickups.clear();
    while ( getline(in, line) ) {
        cnt++;
        // skip comment lines
        if (line[0] == '#') continue;

        Trashnode node(line);  //create node from line on file
        if ( not node.isvalid() or not node.ispickup()) {
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
}







