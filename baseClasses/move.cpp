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

#include <iostream>
#include "move.h"

bool Move::operator==(const Move &rhs) const {
    return nid1==rhs.nid1 and nid2==rhs.nid2 and
           vid1==rhs.vid1 and vid2==rhs.vid2 and
           pos1==rhs.pos1 and pos2==rhs.pos2;
}

bool Move::less(const Move& m) const {
    return nid1<m.nid1 or ( nid1==m.nid1 and
            ( nid2<m.nid2 or ( nid2==m.nid2 and
              ( vid1<m.vid1 or ( vid1==m.vid1 and
                ( pos1<m.pos1 or ( pos1==m.pos1 and pos2<m.pos2 )))))));
}

// The isForbidden() test is not associative ie: A.equiv(B) != B.equiv(A)
// So this test should be used to determine if A would be tabu
// if B is on the tabu list

bool Move::isForbidden(const Move &tabu) const {
    if (*this == tabu) return true;

    // prohibition rules for Ins
    // PR5 - move removing any order from tabu.vid2
    // This rule basically says if we remove a node from vid2
    // then we are not allowed to add a node back to vid2 until
    // the tabu length expires
    // This rule is to promote the elimiation of vehicles
    if (mtype == Ins) {
        if (vid1 == tabu.vid2) return true;
    }
    // prohibition rules for IntraSw
    // This rules says that if we have swapped either the source
    // or the destination nodes that they can not be moved again
    // util the tabu length expires
    else if (mtype == IntraSw) {
        if (nid1==tabu.nid1 or nid2==tabu.nid2 or
            nid1==tabu.nid2 or nid2==tabu.nid1 ) return true;
    }
    // prohibition rules for InterSw
    // This rules says that if we have swapped either the source
    // or the destination nodes that they can not be moved again
    // util the tabu length expires
    else {
        if (nid1==tabu.nid1 or nid2==tabu.nid2 or
            nid1==tabu.nid2 or nid2==tabu.nid1 ) return true;
    }
    return false;
}

void Move::dump() const {
    std::cout << "Move: " << mtype
              << ",\t" << nid1
              << ",\t" << nid2
              << ",\t" << vid1
              << ",\t" << vid2
              << ",\t" << pos1
              << ",\t" << pos2
              << ",\t" << savings
              << std::endl;
}

void Move::Dump() const {
    switch (mtype){
    case Ins:
    	std::cout << "Move: INS" 
              << "\t    NodeID:" << nid1
              << "\tFrom Truck:" << vid1
              << "\t  From Pos:" << pos1
              << "\t  To Truck:" << vid2
              << "\t    To Pos:" << pos2
              << "\t   savings:" << savings
              << std::endl;
    	break;
    case IntraSw:
        std::cout << "Move: IntraSw" 
              << "\t    NodeID:" << nid1
              << "\t  At Truck:" << vid1
              << "\t  From Pos:" << pos1
              << "\t    To Pos:" << pos2
              << "\t   savings:" << savings
              << std::endl;
        break;
    case InterSw:
        std::cout << "Move: InterSw" 
              << "\t    NodeID:" << nid1
              << "\t  (at Truck:" << vid1
              << "\t  at Pos:" << pos1
              << ")\t with NodeID:" << nid2
              << "\t    (at Truck:" << vid2
              << "\t      at Pos:" << pos2
              << ")\t   savings:" << savings
              << std::endl;
        break;
    }

}







