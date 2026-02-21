cmake_minimum_required(VERSION 3.20)
project(MavlinkTagController2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Common warning flags (apply to all targets in this tree) ─────────
add_compile_options(
    -Wall
    -Wextra
    -Werror
    -Wno-address-of-packed-member
)

include(CTest)

# ── CPM package manager ──────────────────────────────────────────────
include(cmake/CPM.cmake)

# ── Options to selectively build components ──────────────────────────
option(BUILD_CONTROLLER     "Build the MAVLink tag controller"       ON)
option(BUILD_DECIMATOR      "Build the ZeroMQ→UDP decimator"        ON)
option(BUILD_AIRSPYHF_ZMQ   "Build the Airspy HF+ ZeroMQ receiver" ON)

# ── MAVLink C headers (fetched via CPM) ──────────────────────────────
CPMAddPackage(
    NAME         mavlink
    GITHUB_REPOSITORY mavlink/c_library_v2
    GIT_TAG      ae473c3ad933c02f41b6f7c9178aad2980d35c81
    DOWNLOAD_ONLY YES
)
if(mavlink_ADDED)
    add_library(mavlink INTERFACE)
    target_include_directories(mavlink INTERFACE
        ${mavlink_SOURCE_DIR}
        ${mavlink_SOURCE_DIR}/common
    )
endif()

# ── Shared wire-format (header-only) ────────────────────────────────
add_library(tagtracker_wireformat INTERFACE)
target_include_directories(tagtracker_wireformat INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/shared>
)

if(BUILD_TESTING)
    add_executable(tagtracker_wireformat_tests shared/test_zmq_iq_packet.c)
    target_link_libraries(tagtracker_wireformat_tests PRIVATE tagtracker_wireformat)
    add_test(NAME tagtracker_wireformat_tests COMMAND tagtracker_wireformat_tests)
endif()

# ── Components ───────────────────────────────────────────────────────
if(BUILD_CONTROLLER)
    add_subdirectory(controller)
endif()

if(BUILD_DECIMATOR)
    add_subdirectory(decimator)
endif()

if(BUILD_AIRSPYHF_ZMQ)
    add_subdirectory(airspyhf_zeromq)
endif()
