#pragma once

/**
 * Boost.Process version compatibility layer.
 *
 * Handles differences between Boost >= 1.83 and earlier versions:
 * - Boost >= 1.83: Use boost::process::v1 (to prevent deprecation warnings)
 * - Boost < 1.83:  Use boost::process directly
 */

#include <boost/version.hpp>

#if BOOST_VERSION >= 108300
    #define BOOST_PROCESS_VERSION 1
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wunused-parameter"
    #pragma clang diagnostic ignored "-Wdeprecated-declarations"
    #include <boost/process/v1.hpp>
    #pragma clang diagnostic pop
#else
    #include <boost/process.hpp>
#endif
