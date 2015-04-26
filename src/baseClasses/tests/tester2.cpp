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
#include "node.h"

void test_node_positionAlogSegment() {
    Node v(1.0,1.0);
    Node w(3.0,2.0);
    Node f0(0,0);
    Node p1(0.9,0.9);
    Node p2(2.0,1.5);
    Node p3(1.1,1.1);
    Node p4(3.1,2.1);
    Node f5(4.0,4.0);
    Node f6(2.5,0.0);

    double tol = 0.2;

    std::cout << "f0(0,0): " << f0.positionAlongSegment(v, w, tol) << std::endl;
    std::cout << "p1(0.9,0.9): " << p1.positionAlongSegment(v, w, tol) << std::endl;
    std::cout << "p2(2.0,1.5): " << p2.positionAlongSegment(v, w, tol) << std::endl;
    std::cout << "p3(1.1,1.1): " << p3.positionAlongSegment(v, w, tol) << std::endl;
    std::cout << "p4(3.1,2.1): " << p4.positionAlongSegment(v, w, tol) << std::endl;
    std::cout << "f5(4.0,4.0): " << f5.positionAlongSegment(v, w, tol) << std::endl;
    std::cout << "f6(2.5,0.0): " << f6.positionAlongSegment(v, w, tol) << std::endl;
}


int main( int argc, char **argv ) {

    test_node_positionAlogSegment();
    return 0;
}
