#ifndef TIMER_H
#define TIMER_H

#include <time.h>

class Timer {

//  public:
//    typedef struct timespec {
//        time_t tv_sec;
//        long   tv_nsec;
//    } Time;

  private:
    struct timespec t0;    // start time
    struct timespec t1;    // duration time

  public:
    Timer() { clock_gettime(CLOCK_MONOTONIC_RAW, &t0); };

    inline void restart() { clock_gettime(CLOCK_MONOTONIC_RAW, &t0); };

    inline double duration() {
        clock_gettime(CLOCK_MONOTONIC_RAW, &t1);
        double dt0 = t0.tv_nsec / 1000000000.0 + t0.tv_sec;
        double dt1 = t1.tv_nsec / 1000000000.0 + t1.tv_sec;
        return dt1 - dt0;
    };
};

#endif
