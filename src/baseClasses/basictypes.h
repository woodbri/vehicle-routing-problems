#ifndef SRC_BASECLASSES_BASICTYPES_H_
#define SRC_BASECLASSES_BASICTYPES_H_

#include <stdint.h>

/*! uniform types for all unsigned integers used as:
   - position  POS
   - cycle iterator UINT
   - any kind of ID
*/
///@{
typedef uint64_t POS;
typedef uint64_t UINT;
typedef uint64_t UID;
///@}

/*! @name \bdouble infinity & -infinity */
///@{
double VRP_MAX();
double VRP_MIN();
///@}

#endif
