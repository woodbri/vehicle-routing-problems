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
#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <map>

#include "singleton.h"

/*!
 * \class Config
 *
 * \brief Create a Config object for storing key/value pairs for configuring the code.
 *
 * The Config object creates a singleton object that can store key/value
 * pairs. This is used to store keys that are referred to by the code to
 * configure the behavior of the code.
 *
 * \author Stephen Woodbridge
 * \date 2014-10-26
 *
 */
class Config {
  private:
    std::map<std::string, std::string> data;

  public:
    Config() { data.clear(); };
    ~Config() {};

    double getDouble(const std::string key) const;
    int getInt(const std::string key) const;
    const std::string getString(const std::string key) const;
    std::vector<std::string> getKeys() const;
    void dump(const std::string title) const;
    bool keyExists(const std::string key) const;

    void set(std::string key, double val);
    void set(std::string key, int val);
    void set(std::string key, std::string val);
    void deleteKey(const std::string key);

};

#endif


/*! \var typedef Singleton<Config> OurConfig
 *  \brief A type definition for a our Config object.
 */
typedef Singleton<Config> OurConfig;

/*! \def CONFIG
 *  \brief A marco to make it easier to access the our Config object.
 *
 *  The Config object is instanciated as a global static object and 
 *  can be referenced use the CONFIG macro like:
 *  \code
    CONFIG->method();
 *  \endcode
 */
#define CONFIG OurConfig::Instance()

