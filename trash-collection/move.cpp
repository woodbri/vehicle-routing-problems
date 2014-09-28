
#include <iostream>
#include "move.h"

bool Move::operator==(const Move &rhs) const {
    return nid1==rhs.nid1 and nid2==rhs.nid2 and
           rid1==rhs.rid1 and rid2==rhs.rid2 and
           pos1==rhs.pos1 and pos2==rhs.pos2;
}

bool Move::less(const Move& m) const {
    return nid1<m.nid1 or ( nid1==m.nid1 and
            ( nid2<m.nid2 or ( nid2==m.nid2 and
              ( rid1<m.rid1 or ( rid1==m.rid1 and
                ( pos1<m.pos1 or ( pos1==m.pos1 and pos2<m.pos2 )))))));
}

// The tabuEquiv() test is not associative ie: A.equiv(B) != B.equiv(A)
// So this test should be used to determine if A would be tabu
// if B is on the tabu list

bool Move::tabuEquiv(const Move &tabu) const {
    if (*this == tabu) return true;

    // prohibition rules for Ins (nid2 will be -1 for Ins)
    // PR5 - move removing any order from tabu.rid2
    if (nid2==-1 and tabu.nid2==-1) {
        if (rid1 == tabu.rid2) return true;
    }
    // prohibition rules for IntraSw (rid2 will be -1 for IntraSw)
    else if (rid2==-1 and tabu.rid2==-1) {
        if (nid1==tabu.nid1 or nid2==tabu.nid2 or
            nid1==tabu.nid2 or nid2==tabu.nid1 ) return true;
    }
    // prohibition rules for InterSw
    else {
        if (nid1==tabu.nid1 or nid2==tabu.nid2 or
            nid1==tabu.nid2 or nid2==tabu.nid1 ) return true;
    }
    return false;
}

void Move::dump() const {
    std::cout << "Move: " << nid1
              << ", " << nid2
              << ", " << rid1
              << ", " << rid2
              << ", " << pos1
              << ", " << pos2
              << std::endl;
}
