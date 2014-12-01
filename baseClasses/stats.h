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
#ifndef STATS_H
#define STATS_H

#include <string>
#include <vector>
#include <map>

#include "singleton.h"
#include "timer.h"

/*! \class Stats
 * \brief Provides a general purpose statistics collection class.
 *
 * This class provides a central collection point for statistics collection
 * while the application code is running. This includes counting events,
 * accumulating sums of values during the execution, and a simple dump
 * of the the collected stats as required.
 *
 * All stats are double values associated with std::string keys.
 *
 * When combined with \ref Timer we can for example, sum the duration of
 * time spend it a function and count the number of time the function
 * called.
 *
 * Access to this facility requires:
 *
 * \code
 * #include "stats.h"
 * STATS->method();
 * \endcode
 *
 * where method is one of the various methods documented for the class.
 */
class Stats {
  private:
    std::map<std::string, double> stats;

  public:

    Stats() { stats.clear(); };
    ~Stats() {};

    double getval(const std::string key) const;
    std::vector<std::string> getkeys() const;
    void dump(const std::string title) const;

    void inc(const std::string key);
    void set(const std::string key, double val);
    void addto(const std::string key, double val);
    void clear() { stats.clear(); };
};

typedef Singleton<Stats> VrpStats; // Global declaration

#define STATS VrpStats::Instance()

#endif
