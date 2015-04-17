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

#include <limits>
#include <stdexcept>
#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "timer.h"
//#include "plot.h"
#endif

#include "givenInitialSol.h"






//    PROCESS
//
//    This implements a feasable solution

void givenInitialSol::process()
{
#ifdef DOSTATS
  Timer start;
#endif
// TODO process to make feasable

#ifdef DOSTATS
  DLOG( INFO ) << "GivenInitialSolution: Total time: " << start.duration();
#endif
  return;
}
