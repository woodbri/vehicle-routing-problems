
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
    Testnode( std::string& line ) : Twnode( line ) {};
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

    bool compareNid( std::vector<int> nids ) const {
        if (path.size() != nids.size())
            return false;
        for (int i=0; i<path.size(); i++)
            if (path[i].getnid() != nids[i]) return false;
        return true;
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

class TestProblem {
  public:
    std::vector<Vehicle> fleet;
    Twpath<Testnode> datanodes;
    int cnt;
    int numInValid;
    int numTests;
    int numPass;
    int numFail;

  public:

    // status: 0=OK, 1=EOF, 2=End Of Section
    std::string getNextLine( std::ifstream& in, int *status ) {
        std::string line;

        *status = 0;

        while (std::getline(in, line)) {
            cnt++;
            if (line[0] == '#') continue;
            if (line[0] == '-' and line[1] == '1')
                *status = 2;
            return line;
        }
        *status = 1;
        return line;
    };


    bool runTest( const Test& t ) {
        if (t.path_id < 0 or t.path_id > fleet.size()-1) {
            std::cout << "ERROR: test: " << t.test_id << " requested path ("
                      << t.path_id << ") which is unknown!" << std::endl;
            return false;
        }

        if (t.test_name.compare("e_move3") == 0) {
            Vehicle v = fleet[t.path_id];
            v.path.e_move(t.args[0], t.args[1], t.args[2]);
            if (t.path_result.size() and v.compareNid(t.path_result)) {
                std::cout << "ERROR: test: " << t.test_id
                    << " path results do not match!" << std::endl;
                std::cout << "  expected: "; v.dumpnids();
                std::cout << "       got: "; t.dumpnids();
                return false;
            }
            if (t.checkCosts and
                (t.cost != v.getcost() or t.twv != v.getTWV() or
                    t.cv != v.getCV())) {
                std::cout << "ERROR: test: " << t.test_id
                    << " cost and violations do not match!" << std::endl;
                std::cout << "  expected: " << t.cost << ", " << t.twv 
                    <<", " << t.cv <<std::endl;
                std::cout << "       got: " << v.getcost() << ", " << v.getTWV()
                    <<", " << v.getCV() <<std::endl;
                return false;
            }
        }
        else if (t.test_name.compare("e_move4") == 0) {
        }
        else {
            std::cout << "ERROR: test: " << t.test_id << " requested test ("
                      << t.test_name << ") which is unknown!" << std::endl;
            return false;
        }

        return true;
    };

/*
# tests
# -------------------------------
# test_id path_id function arg1 arg2 ...
# 0 path nids
# 1 cost twv cv
# -1
0 0 e_move3 4 9 1000
0 0 1 2 3 5 6 7 8 9 4
-1
1 0 e_move3 4 0 1000
0 4 0 1 2 3 5 6 7 8 9
-1
2 0 e_move3 4 20 1000
0 0 1 2 3 4 5 6 7 8 9
-1
*/

    Test loadTest( std::ifstream& in ) {
        Test t;
        int status;
        int rtype;
        double dval;
        std::string line;

        // get the first line of the test
        line = getNextLine( in , &status );
        if (status) {
            t.isEof = true;
            return t;
        }

        std::istringstream buffer( line );
        buffer >> t.test_id;
        buffer >> t.path_id;
        buffer >> t.test_name;
        while (buffer >> dval)
            t.args.push_back(dval);

        status = 0;
        while (!status) {
            // get the next line of the test
            line = getNextLine( in , &status );
            if (status) {
                if (status == 1)
                    t.isEof = true;
                return t;
            }

            std::istringstream buffer( line );
            buffer >> rtype;

            switch (rtype) {
                case 0:
                    int nid;
                    while (buffer >> nid)
                        t.path_result.push_back( nid );
                    t.isTestValid = true;
                    break;
                case 1:
                    buffer >> t.cost;
                    buffer >> t.twv;
                    buffer >> t.cv;
                    t.checkCosts = true;
                    t.isTestValid = true;
                    break;
                default:
                    break;
            }
        }

        return t;
    };


    int getnumtests() const { return numTests; };


    int getnumpass() const { return numPass; };


    TestProblem(std::string& file) {
        std::ifstream in( file.c_str() );
        std::string line;

        datanodes.clear();

        numInValid = numTests = numPass = numFail = 0;

        cnt = 0;

        // read the nodes
        while ( std::getline(in, line) ) {
            cnt++;
            // skip comment lines
            if (line[0] == '#') continue;
            // end of section if -1
            if (line[0] == '-' and line[1] == '1') break;

            Testnode node( line );
            if (!node.isvalid())
                std::cout << "ERROR: line: " << cnt << ": " << line << std::endl;
            datanodes.push_back(node);
        }

        // read and construct the paths
        while ( std::getline(in, line) ) {
            cnt++;
            // skip comment lines
            if (line[0] == '#') continue;
            // end of section if -1
            if (line[0] == '-' and line[1] == '1') break;

            Vehicle truck( datanodes, line );

            fleet.push_back( truck );
        }

        // read and run tests
        while ( ! in.eof() ) {
            Test t = loadTest( in );

            if (t.eof()) break;

            t.dump();
            numTests++;

            if (! t.isValid()) {
                std::cout << "Test: " << t.getid() << " is not valid!\n";
                numInValid++;
                continue;
            }

            if (runTest( t ))
                numPass++;
            else
                numFail++;
        }

        in.close();

        std::cout << "Number Tests: " << numTests << std::endl;
        std::cout << "Number Pass: " << numPass << std::endl;
        std::cout << "Number Fail: " << numFail << std::endl;
        std::cout << "Number Invalid: " << numInValid << std::endl;
    };
};

void Usage() {
    std::cout << "Usage: tester in.txt\n";
}

int main(int argc, char **argv) {

    if (argc < 2) {
        Usage();
        return 1;
    }

    std::string infile = argv[1];

    try {

        TestProblem tp( infile );
        if (tp.getnumtests() != tp.getnumpass())
            return 1;

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
