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
#ifndef SRC_BASECLASSES_TIMER_H_
#define SRC_BASECLASSES_TIMER_H_

#include <time.h>

/*! \class Timer
 * \brief Create a Timer object for measuring the time duration from the Timer creation.
 *
 * A simple Timer class that can be used to report the the duration of time
 * since it was created. It is used to measure how long certain functions
 * take to run and for collecting performance stats.
 */
class Timer
{

private:
  struct timespec startTime;   
  struct timespec stopTime;

public:
  /*! \brief Construct a new Timer object and remember its creation time*/
  Timer() { clock_gettime( CLOCK_MONOTONIC_RAW, &startTime ); };

  /*! \brief Resets the Timer start time to now.  */
  inline void restart() { clock_gettime( CLOCK_MONOTONIC_RAW, &startTime ); };

  /*! \brief Returns the time lapse since the Timer was set or reset.  */
  inline double duration() {
    clock_gettime( CLOCK_MONOTONIC_RAW, &stopTime );
    double dt0 = startTime.tv_nsec / 1000000000.0 + startTime.tv_sec;
    double dt1 = stopTime.tv_nsec / 1000000000.0 + stopTime.tv_sec;
    return dt1 - dt0;
  };
};

#endif  // SRC_BASECLASSES_TIMER_H_
