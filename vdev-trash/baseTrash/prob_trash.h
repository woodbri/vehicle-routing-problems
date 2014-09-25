#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

//#include "order.h"
//#include "orders.h"
#include "bucketn.h"
#include "vehicle.h"
#include "twpath.h"
#include "twc.h"

class Prob_trash {
  protected:
typedef  TwBucket<Trashnode> Bucket;

    Trashnode depot;

    TWC<Trashnode> twc;
    Twpath<Trashnode> datanodes; //dissallowing set operations
    Bucket dumps;
    Bucket depots;
    Bucket pickups;
    Bucket invalid;
    std::deque<Vehicle> trucks; 
    std::deque<Vehicle> invalidTrucks; 

    std::string datafile;


  public:

    Trashnode getdepot() const { return depot;};
    void loadProblem(char *infile);
    Prob_trash(char *infile);

    unsigned int getNodeCount() const;

    bool checkIntegrity() const;


    double distance(int n1, int n2) const;
    double nodeDemand(int i) const;
    double nodeServiceTime(int i) const;
    bool earlyArrival(int nid,double D) const; 
    bool lateArrival(int nid,double D) const; 

    void twcijDump() const;



    void nodesdump();
    void nodesdumpeval();
    void plot(Plot<Trashnode> &graph);
    void dump();


    inline double _MAX() { (std::numeric_limits<double>::max()); };
    inline double _MIN() { ( - std::numeric_limits<double>::max() ); };

private:
    void load_depots(std::string infile, int &nid);
    void load_dumps(std::string infile, int &nid);
    void load_pickups(std::string infile,int &nid);
    void load_trucks(std::string infile);
    void load_matrix(std::string infile);
};

#endif
