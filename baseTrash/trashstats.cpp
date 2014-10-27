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

#include "trashstats.h"


double TrashStats::getval(const std::string key) const {
    std::map<const std::string, double>::const_iterator it;
    it = stats.find(key);
    if (it == stats.end())
        return 0.0;
    else
        return it->second;
}


std::vector<std::string> TrashStats::getkeys() const {
    std::vector<std::string> keys;
    std::map<std::string, double>::const_iterator it;

    for (it = stats.begin(); it!=stats.end(); ++it)
        keys.push_back(it->first);

    return keys;
}


void TrashStats::dump(const std::string title) const {
    std::map<std::string, double>::const_iterator it;

    std::cout << "---------- TrashStats: " << title
              << " --------------" << std::endl;
    for (it = stats.begin(); it!=stats.end(); ++it)
        std::cout << it->first << ":\t" << it->second << std::endl;

    std::cout << "----------------------------------------" << std::endl;
}


void TrashStats::inc(const std::string key) {
    std::map<std::string, double>::iterator it;
    it = stats.find(key);
    if (it == stats.end())
        stats[key] = 1.0;
    else
        stats[key] = stats[key] + 1.0;
}


void TrashStats::set(const std::string key, double val) {
    stats[key] = val;
}


void TrashStats::addto(const std::string key, double val) {
    std::map<std::string, double>::iterator it;
    it = stats.find(key);
    if (it == stats.end())
        stats[key] = val;
    else
        stats[key] = stats[key] + val;
}


