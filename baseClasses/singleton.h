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
#ifndef SINGLETON_H
#define SINGLETON_H

#include <stddef.h>  // defines NULL
#include <cassert>

/*! \class Singleton
 * \brief A template class for creating  singleton objects.
 * This class if used by \ref Config class and  \ref TrashStats class.
 *
 * An example of how to use this class to create a simple Logger class:
 *
 * \code
    // USAGE:
    //   Here is an example of how to use this class to create a logger

    #include "singleton.h"

    class Logger
    {
    public:
       Logger() {};
       ~Logger() {};
       bool openLogFile(string);
       void writeToLogFile(string);
       bool closeLogFile(string);
    private:
       ...
       ...
    };

    bool Logger::openLogFile(std::string)
    {
       ...
       ...
    }

    ...
    ...

    // create the singletons object in as a global
    typedef Singleton<Logger> LoggerSingleton;   // Global declaration

    main()
    {
        ...

        // Use this to access the global object
        // it is often convenient to
        LoggerSingleton::Instance()->openLogFile("logFile.txt");

        // #define LOG LoggerSingleton::Instance()
        // LOG->openLogFile("logFile.txt");

        ...
    }
 * \endcode
 */
template <class T>
class Singleton {
  public:
    static T *Instance() {
        if ( !m_pInstance ) m_pInstance = new T;

        assert( m_pInstance != NULL );
        return m_pInstance;
    }
  protected:
    Singleton();
    ~Singleton();
  private:
    Singleton( Singleton const & );
    Singleton &operator=( Singleton const & );
    static T *m_pInstance;
};

template <class T> T *Singleton<T>::m_pInstance = NULL;

#endif
/*

*/
