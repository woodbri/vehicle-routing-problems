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
#include "twpath.h"
#include "tweval.h"

/*
# paths
# -------------------------------
# path_id nid1 nid2 nid3 ...
0 0 1 2 3 4 5 6 7 8 9
1 10 11 12 13 14 15 16 17 18 19
-1
*/

class Testnode : public Tweval {
  public:
    Testnode( std::string& line ) : Tweval() {
        std::istringstream buffer( line );
        buffer >> nid;
        buffer >> x;
        buffer >> y;
        buffer >> demand;
        buffer >> tw_open;
        buffer >> tw_close;
        //buffer >> service;
    };
};


class Vehicle {
  public:
    Twpath<Testnode> path;
    int maxcapacity;
    double cost;        // cost of the route
    double w1;          // weight for duration in cost
    double w2;          // weight for TWV in cost
    double w3;          // weight for CV in cost

  public:
    Vehicle(){};
    Vehicle( const Twpath<Testnode>& datanodes, std::string& line ) {
        int id;
        int nid;
        maxcapacity = 0;
        cost        = 0;
        w1 = w2 = w3 = 1.0;
        std::istringstream buffer( line );
        buffer >> id;
        while ( ! buffer.eof() ) {
            buffer >> nid;
            path.push_back( datanodes[nid] );
        }
    };

    double getcost() const { return cost; };
    int getTWV() const { return 0; /* TODO */ };
    int getCV() const { return 0; /* TODO */ };

    // return true on error
    bool compareNid( std::vector<int> nids ) const {
        if (path.size() != nids.size())
            return true;
        for (int i=0; i<path.size(); i++)
            if (path[i].getnid() != nids[i]) return true;
        return false;
    }

    void dumpnids() const {
        for (int i=0; i<path.size(); i++)
            std::cout << path[i].getnid() << ", ";
        std::cout << std::endl;
    }

};

class Test {
  public:
    int test_id;
    int path_id;
    std::string test_name;
    std::vector<double> args;
    std::vector<int> path_result;
    double cost;
    int twv;
    int cv;
    bool checkCosts;
    bool isTestValid;
    bool isEof;

  public:
    Test() {
        test_id = -1;
        path_id = -1;
        test_name = "NONE";
        args.clear();
        path_result.clear();
        cost = -1;
        twv = -1;
        cv = -1;
        checkCosts = false;
        isTestValid = false;
        isEof = false;
    };

    bool eof() const { return isEof; };
    bool isValid() const { return isTestValid; };
    int getid() const { return test_id; };

    void dumpnids() const {
        for (int i=0; i<path_result.size(); i++)
            std::cout << path_result[i] << ", ";

        std::cout << std::endl;
    };

    void dump() const {
        std::cout << "--- Test Case ----" << std::endl
                  << "test_id: " << test_id << std::endl
                  << "path_id: " << path_id << std::endl
                  << "test_name: " << test_name << std::endl
                  << "args: ";
        for (int i=0; i<args.size(); i++)
            std::cout << args[i] << ", ";
        std::cout << std::endl;
        std::cout << "path_result: "; dumpnids();
        std::cout << "cost: " << cost << std::endl
                  << "twv: " << twv << std::endl
                  << "cv: " << cv << std::endl
                  << "checkCosts: " << checkCosts << std::endl
                  << "isValid: " << isTestValid << std::endl
                  << "isEof: " << isEof << std::endl;
    };

};
