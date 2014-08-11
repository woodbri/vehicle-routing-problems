
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


void Usage() {
    std::cout << "Usage: tester in.txt\n";
}

int main(int argc, char **argv) {

    if (argc < 2) {
        Usage();
        return 1;
    }

    char * infile = argv[1];

    try {

        Node n;
        n.setvalues(1, 10, 20);
        n.dump();

        Node n2(2, 11,21);
        n2.dump();

        Twnode tw;
        tw.setvalues(2, 20, 30, 40, 120, 600, 5);
        tw.dump();

        Trashnode tn;
        tn.setvalues(3, 21, 32, 0, 40, 120, 600, 5);
        tn.dump();

    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
