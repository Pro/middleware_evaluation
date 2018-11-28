//
// Created by profanter on 9/3/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_TIME_UTILS_H
#define PROJECT_TIME_UTILS_H

#include <time.h>

int
timeval_subtract (struct timespec *result, struct timespec *x, struct timespec *y)
{
    /* Perform the carry for the later subtraction by updating y. */
    if (x->tv_nsec < y->tv_nsec) {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        y->tv_nsec -= 1000000000 * nsec;
        y->tv_sec += nsec;
    }
    if (x->tv_nsec - y->tv_nsec > 1000000000) {
        int nsec = (x->tv_nsec - y->tv_nsec) / 1000000000;
        y->tv_nsec += 1000000000 * nsec;
        y->tv_sec -= nsec;
    }

    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_nsec = x->tv_nsec - y->tv_nsec;

    /* Return 1 if result is negative. */
    return x->tv_sec < y->tv_sec;
}

// -Method to get second resolution timestamp
double time_measure_get_seconds(){
    struct timespec ts2;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts2);
    return ts2.tv_sec;
}

#define TIME_MEASURE_START(X) clock_gettime(CLOCK_MONOTONIC_RAW, &X)
#define TIME_MEASURE_DIFF_USEC(X, TARGET) { \
    struct timespec _diff_time_end, _diff_time; \
    clock_gettime(CLOCK_MONOTONIC_RAW, &_diff_time_end); \
    timeval_subtract(&_diff_time, &_diff_time_end, &X); \
    (TARGET) = _diff_time.tv_sec * 1000000 + _diff_time.tv_nsec / 1000; \
    }

#endif //PROJECT_TIME_UTILS_H
