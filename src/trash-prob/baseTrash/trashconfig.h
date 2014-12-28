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
#ifndef TRASHCONFIG_H
#define TRASHCONFIG_H

#include <string>

#include "config.h"

class TrashConfig : public Config
{
public:

  TrashConfig();
  ~TrashConfig() {};
};
#endif

/*! \var typedef Singleton<TrashConfig> OurTrashConfig
 *  \brief A type definition for a our Config object.
 */
typedef Singleton<TrashConfig> OurTrashConfig;

/*! \def CONFIG
 *  \brief A macro to make it easier to access the our TrashConfig object.
 *
 *  The TrashConfig object is instanciated as a global static object and
 *  can be referenced using the CONFIG macro like:
 *  \code
    CONFIG->method();
 *  \endcode
 */
// redefine baseClasses CONFIG to access TrashConfig
#ifdef CONFIG
#undef CONFIG
#endif
#define CONFIG OurTrashConfig::Instance()

