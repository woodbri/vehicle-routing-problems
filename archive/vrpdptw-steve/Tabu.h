#ifndef TABU_H
#define TABU_H

#include "Problem.h"
#include "Move.h"

class Tabu {
  public:
    int node;
    int torid;
    int topos;
    int expires;
    int checked;
    int aspirational;
    int move;

    Tabu() {
        node  = -1;
        torid = -1;
        topos = -1;
        expires = 0;
        checked = 0;
        aspirational = 0;
        move = -1;
    };

    inline void dump() {
        std::cout << "TABU: node: " << node
                  << ", torid: "    << torid
                  << ", topos: "    << topos
                  << ", expires: "  << expires
                  << ", checked: "  << checked
                  << ", aspirational: " << aspirational
                  << ", move: " << move
                  << std::endl;
    };

}; // end of class


inline bool operator==(const Tabu& a, const Tabu& b) {
    return a.node == b.node
            && a.torid == b.torid
            // if either are -1 then we don't care about position
            && (a.topos == b.topos || a.topos == -1 || b.topos == -1);
};

#endif
