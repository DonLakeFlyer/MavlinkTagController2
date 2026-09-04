#pragma once

// Always-on test check. The controller test executables must fail loudly in
// Release builds too, where CMake defines NDEBUG and turns assert() into a
// no-op (and -Werror then flags the now-unused test variables).

#include <cstdio>
#include <cstdlib>

#define CHECK(expr)                                                              \
    do {                                                                         \
        if (!(expr)) {                                                           \
            std::fprintf(stderr, "CHECK failed: %s (%s:%d)\n", #expr, __FILE__,  \
                         __LINE__);                                              \
            std::abort();                                                        \
        }                                                                        \
    } while (0)
