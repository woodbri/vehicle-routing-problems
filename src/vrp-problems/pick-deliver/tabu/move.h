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
#ifndef MOVE_H
#define MOVE_H

class Move {
  public:
    int moveType;
    int oid1;
    int oid2;
    int rid1;
    int rid2;
    int ppos1;
    int spos1;
    int ppos2;
    int spos2;
    double savings;

    Move() {
        moveType = -1; // 1-SPI, 2-SBR, 3=WRI
        oid1 = -1;
        oid2 = -1;
        rid1 = -1;
        rid2 = -1;
        ppos1 = -1;
        spos1 = -1;
        ppos2 = -1;
        spos2 = -1;
        savings = 0;
    };

    inline void dump() const {
        std::cout << "Move: type: " << moveType
                  << ", oid1: " << oid1
                  << ", oid2: " << oid2
                  << ", rid1: " << rid1
                  << ", rid2: " << rid2
                  << ", ppos1: " << ppos1
                  << ", spos1: " << spos1
                  << ", ppos2: " << ppos2
                  << ", spos2: " << spos2
                  << ", savings: " << savings
                  << std::endl;
    };

};


// == operator for use with std::find
// used to search if a move is on the tabu list

inline bool operator==(const Move& a, const Move& b) {
    return a.moveType == b.moveType &&
        a.oid1 == b.oid1 && a.oid2 == b.oid2 &&
        a.rid1 == b.rid1 && a.rid2 == b.rid2;
};

#endif
