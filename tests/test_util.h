#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Minimal assert-style test helpers. Each test file is its own executable;
// a failed check prints the location and exits non-zero (CTest reports it).

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::fprintf(stderr, "CHECK failed: %s (%s:%d)\n",             \
                         #cond, __FILE__, __LINE__);                       \
            std::exit(1);                                                  \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
    do {                                                                   \
        const double _a = (a), _b = (b), _t = (tol);                       \
        if (std::abs(_a - _b) > _t) {                                      \
            std::fprintf(stderr,                                          \
                         "CHECK_NEAR failed: %s=%.9g vs %s=%.9g "          \
                         "(tol %.3g) (%s:%d)\n",                           \
                         #a, _a, #b, _b, _t, __FILE__, __LINE__);          \
            std::exit(1);                                                  \
        }                                                                  \
    } while (0)
