/* Minimal test harness for libcari */
#ifndef CARI_T_H
#define CARI_T_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int t_fail, t_checks;

#define CHECK(cond) do {                                                     \
    t_checks++;                                                              \
    if (!(cond)) {                                                           \
        t_fail++;                                                            \
        fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    }                                                                        \
} while (0)

#define CHECK_EQ(a, b) do {                                                  \
    long long a_ = (long long)(a), b_ = (long long)(b);                      \
    t_checks++;                                                              \
    if (a_ != b_) {                                                          \
        t_fail++;                                                            \
        fprintf(stderr, "%s:%d: CHECK_EQ failed: %s (%lld) != %s (%lld)\n",  \
                __FILE__, __LINE__, #a, a_, #b, b_);                         \
    }                                                                        \
} while (0)

#define CHECK_NEAR(a, b, eps) do {                                           \
    double a_ = (double)(a), b_ = (double)(b);                               \
    t_checks++;                                                              \
    if (!(fabs(a_ - b_) <= (eps))) {                                         \
        t_fail++;                                                            \
        fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s (%g) vs %s (%g)\n",    \
                __FILE__, __LINE__, #a, a_, #b, b_);                         \
    }                                                                        \
} while (0)

#define RUN(fn) do { fprintf(stderr, "-- %s\n", #fn); fn(); } while (0)

#define T_DONE() do {                                                        \
    fprintf(stderr, "%d checks, %d failed\n", t_checks, t_fail);             \
    return t_fail ? 1 : 0;                                                   \
} while (0)

#endif
