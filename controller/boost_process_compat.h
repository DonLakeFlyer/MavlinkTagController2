#pragma once

/**
 * Boost.Process compatibility wrapper.
 * - Boost >= 1.84 ships v2 as the default; explicitly request v1.
 * - Boost < 1.84 only has v1 via <boost/process.hpp>.
 * - Suppresses deprecation and unused-parameter warnings on GCC and Clang.
 */

#include <boost/version.hpp>

#if defined(__clang__)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wunused-parameter"
    #pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#if BOOST_VERSION >= 108400
    #include <boost/process/v1.hpp>
#else
    #include <boost/process.hpp>
#endif

#if defined(__clang__) || defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif

// Provide a unified namespace alias: v1 types live under boost::process::v1
// in Boost >= 1.84, but directly under boost::process in older versions.
#if BOOST_VERSION >= 108400
    namespace bp = boost::process::v1;
#else
    namespace bp = boost::process;
#endif
