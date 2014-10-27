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

template <class T>
class Singleton
{
public:
  static T* Instance() {
      if(!m_pInstance) m_pInstance = new T;
      assert(m_pInstance != NULL);
      return m_pInstance;
  }
protected:
  Singleton();
  ~Singleton();
private:
  Singleton(Singleton const&);
  Singleton& operator=(Singleton const&);
  static T* m_pInstance;
};

template <class T> T* Singleton<T>::m_pInstance=NULL;

#endif
/*

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

typedef Singleton<Logger> LoggerSingleton;   // Global declaration

main()
{
    ...

    LoggerSingleton::Instance()->openLogFile("logFile.txt");

    ...
}
*/
