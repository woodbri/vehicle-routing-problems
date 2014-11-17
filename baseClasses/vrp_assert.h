
#ifndef VRP_ASSERT_H
#define VRP_ASSERT_H

#include <exception>

#ifdef assert
#undef assert
#endif

#define assert(expr) \
    ((expr) \
     ? static_cast<void>(0) \
     : __assert_failed( __STRING(expr), __FILE__,  __LINE__ ,  __PRETTY_FUNCTION__ ))

#define __assert_failed(a, b, c, d) \
    throw new AssertFailedException( "AssertFailedException: " #a ", " #b ", " #c ", " #d )

class AssertFailedException : public std::exception {
  private:
    const char * str;

  public:
    virtual const char * what() const throw() {
        return str;
    };

    AssertFailedException(const char * _str) : str(_str) {};

};

#endif
