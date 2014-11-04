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
#ifndef TIMER_H
#define TIMER_H

#include <time.h>

/*! \class Timer
 * \brief Create a Timer object for measuring the time duration from the Timer creation.
 *
 * A simple Timer class that can be used to report the the duration of time
 * since it was created. It is used to measure how long certain functions
 * take to run and for collecting performance stats.
 */
class Timer {

    //  public:
    //    typedef struct timespec {
    //        time_t tv_sec;
    //        long   tv_nsec;
    //    } Time;

  private:
    struct timespec t0;    ///< start time
    struct timespec t1;    ///< duration time

  public:
    /*!
     * \brief Construct a new Timer object and remember its time of creation
     */
    Timer() { clock_gettime( CLOCK_MONOTONIC_RAW, &t0 ); };

    /*!
     * \brief Reset the Timer start time to now.
     */
    inline void restart() { clock_gettime( CLOCK_MONOTONIC_RAW, &t0 ); };

    /*!
     * \brief Compute the duration of time since the Timer was set or reset.
     * \return The number of second as a double since the Timer as set or reset.
     */
    inline double duration() {
        clock_gettime( CLOCK_MONOTONIC_RAW, &t1 );
        double dt0 = t0.tv_nsec / 1000000000.0 + t0.tv_sec;
        double dt1 = t1.tv_nsec / 1000000000.0 + t1.tv_sec;
        return dt1 - dt0;
    };
};

#endif
