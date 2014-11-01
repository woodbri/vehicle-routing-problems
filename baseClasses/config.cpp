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

#include <cstdlib>
#include <cstdio>
#include <iostream>

#include "config.h"


/*!
 * \brief Fetch config value associated with key as a double value.
 *
 * Config values are stored as stings. This function converts
 * the string to a double. If the string is not a number or
 * if the key does not exist then 0.0 will be returned.
 *
 * \param[in] key The config key for the desired value.
 * \return The string value converted to a double or 0.0
 * \warning set(double) converts double values with only 6 digits percision.
 *
 */
double Config::getDouble( const std::string key ) const {
    std::map<std::string, std::string>::const_iterator it;
    it = data.find( key );

    if ( it == data.end() )
        return 0.0;
    else
        return strtod( it->second.c_str(), NULL );
}


/*!
 * \brief Fetch config value associated with key as a int value.
 *
 * Config values are stored as stings. This function converts
 * the string to a int. If the string is not a number or
 * if the key does not exist then 0 will be returned.
 *
 * \param[in] key The config key for the desired value.
 * \return The string value converted to a integer or 0
 *
 */
int Config::getInt( const std::string key ) const {
    std::map<std::string, std::string>::const_iterator it;
    it = data.find( key );

    if ( it == data.end() )
        return 0;
    else
        return strtol( it->second.c_str(), NULL, 10 );
}


/*!
 * \brief Fetch the string value associated with key.
 *
 * If the key does not exist then a null string is returned.
 *
 * \param[in] key The config key for the desired value.
 * \return The string value associated with the key or a null string.
 *
 */
const std::string Config::getString( const std::string key ) const {
    std::map<std::string, std::string>::const_iterator it;
    it = data.find( key );

    if ( it == data.end() )
        return "";
    else
        return it->second;
}


/*!
 * \brief Fetch a vector containing the key strings.
 *
 * Fetch all the keys defined in the config map and return them
 * in a vector.
 *
 * \return A string vector with all defined keys in it.
 *
 */
std::vector<std::string> Config::getKeys() const {
    std::vector<std::string> keys( data.size() );
    std::map<std::string, std::string>::const_iterator it;

    for ( it = data.begin(); it != data.end(); ++it )
        keys.push_back( it->first );

    return keys;
}


/*!
 * \brief Print the contents of the Config object.
 *
 */
void Config::dump( const std::string title ) const {
    std::map<std::string, std::string>::const_iterator it;

    std::cout << "---------- CONFIG -------------" << std::endl;

    for ( it = data.begin(); it != data.end(); ++it )
        std::cout << it->first << ": '" << it->second << "'" << std::endl;

    std::cout << "-------------------------------" << std::endl;
}


/*!
 * \brief Check if a key exists in the config data store.
 *
 * \return Returns true if the key exists in the config data store, otherwise false.
 *
 */
bool Config::keyExists( const std::string key ) const {
    std::map<std::string, std::string>::const_iterator it;
    it = data.find( key );
    return it != data.end();
}


/*!
 * \brief Associate double val with key in the config data store.
 *
 * \param[in] key The config key that the val should be associated with.
 * \param[in] val A double value to be converted to a string and associated with key.
 * \warning val is converted to a string with only 6 decimals of percision for storage in the config data store.
 *
 */
void Config::set( std::string key, double val ) {
    char buf[50];
    snprintf( buf, 50, "%e", val );
    data[key] = std::string( buf );
}


/*!
 * \brief Associate integer val with key in the config data store.
 *
 * \param[in] key The config key that the val should be associated with.
 * \param[in] val An integer value to be converted to a string and associated with key.
 *
 */
void Config::set( std::string key, int val ) {
    char buf[50];
    snprintf( buf, 50, "%d", val );
    data[key] = std::string( buf );
}


/*!
 * \brief Associate a string val with key in the config data store.
 *
 * \param[in] key The config key that the val should be associated with.
 * \param[in] val The value to be associated with key.
 *
 */
void Config::set( std::string key, std::string val ) {
    data[key] = val;
}


/*!
 * \brief Remove the key and associated value from the config data store..
 *
 */
void Config::deleteKey( const std::string key ) {
    std::map<std::string, std::string>::iterator it;
    it = data.find( key );

    if ( it != data.end() )
        data.erase( it );
}


