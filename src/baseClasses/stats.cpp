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

#include "logger.h"
#include "stats.h"


/*!
 * \brief Fetch the current value of \b key
 * \param[in] key The key that we want to retrieve the value of.
 * \return The current value of \b key if it is defined or 0.0.
 */
double Stats::getval( const std::string key ) const
{
  std::map<const std::string, double>::const_iterator it;
  it = stats.find( key );

  if ( it == stats.end() )
    return 0.0;
  else
    return it->second;
}


/*!
 * \brief Fetch all keys into a std::vector of std::strings.
 * \return A std::vector of std::strings of all the defined keys.
 */
std::vector<std::string> Stats::getkeys() const
{
  std::vector<std::string> keys;
  std::map<std::string, double>::const_iterator it;

  for ( it = stats.begin(); it != stats.end(); ++it )
    keys.push_back( it->first );

  return keys;
}


/*!
 * \brief Print out all the key: value pairs currently in the Stats object.
 * \param[in] title A std::string title that will get printed along with the output.
 */
void Stats::dump( const std::string title ) const
{
  std::map<std::string, double>::const_iterator it;

  DLOG( INFO ) << "---------- Stats: " << title << " --------------";

  for ( it = stats.begin(); it != stats.end(); ++it )
    DLOG( INFO ) << it->first << ":\t" << it->second;

  DLOG( INFO ) << "----------------------------------------";
}


/*!
 * \brief Increment (or initialize) the value associated with the key.
 * \param[in] key The key we want to increment.
 */
void Stats::inc( const std::string key )
{
  std::map<std::string, double>::iterator it;
  it = stats.find( key );

  if ( it == stats.end() )
    stats[key] = 1.0;
  else
    stats[key] = stats[key] + 1.0;
}


/*!
 * \brief Set a given \b key to a given value.
 *
 * Set the \b key to \b val. In general the class will automatically initialize
 * a key if it does not exist. This method is for explicitly setting the key
 * to a value, for example, if you want to save some important state and report
 * it in the stats.
 *
 * \param[in] key The key as a std::string that we want to assign \b val to.
 * \param[in] val The value as a double that we want associated to \b key.
 */
void Stats::set( const std::string key, double val )
{
  stats[key] = val;
}


/*!
 * \brief Add a value to a key to accumulate a sum.
 * \param[in] key The key as a std::string that we want to assign \b val to.
 * \param[in] val The value as a double that we want associated to \b key.
 */
void Stats::addto( const std::string key, double val )
{
  std::map<std::string, double>::iterator it;
  it = stats.find( key );

  if ( it == stats.end() )
    stats[key] = val;
  else
    stats[key] = stats[key] + val;
}


