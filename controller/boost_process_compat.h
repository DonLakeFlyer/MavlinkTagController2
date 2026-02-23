#pragma once

/**
 * Boost.Process compatibility wrapper.
 * Suppresses deprecation and unused-parameter warnings from Boost headers
 * on both GCC and Clang.
 */

#if defined(__clang__)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wunused-parameter"
    #pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <boost/process.hpp>

#if defined(__clang__) || defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif
