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
#ifndef SRC_BASECLASSES_BASICTYPES_H_
#define SRC_BASECLASSES_BASICTYPES_H_

#include <stdint.h>

/*! uniform types for all unsigned integers used as:
   - position  POS
   - cycle iterator UINT
   - any kind of ID
*/
///@{
typedef uint64_t POS;
typedef uint64_t UINT;
typedef uint64_t UID;
///@}

/*! @name \bdouble infinity & -infinity */
///@{
double VRP_MAX();
double VRP_MIN();
///@}

#endif
