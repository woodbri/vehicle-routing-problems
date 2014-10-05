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
