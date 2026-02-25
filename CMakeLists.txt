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

# ── Build dependency preflight (all components are always built) ─────
set(Boost_USE_MULTITHREADED ON)
find_package(Boost QUIET)
if(NOT Boost_FOUND)
    message(FATAL_ERROR
        "Missing dependency: Boost\n"
        "Install development package(s), e.g.:\n"
        "  Ubuntu/Debian: sudo apt install libboost-all-dev\n"
        "  macOS (Homebrew): brew install boost"
    )
endif()

find_package(Threads QUIET)
if(NOT Threads_FOUND)
    message(FATAL_ERROR
        "Missing dependency: POSIX threads (pthreads)\n"
        "This is normally provided by libc; ensure a C toolchain is installed, e.g.:\n"
        "  Ubuntu/Debian: sudo apt install build-essential"
    )
endif()

find_package(PkgConfig QUIET)
if(NOT PKG_CONFIG_FOUND)
    message(FATAL_ERROR
        "Missing dependency: pkg-config\n"
        "Install it, e.g.:\n"
        "  Ubuntu/Debian: sudo apt install pkg-config\n"
        "  macOS (Homebrew): brew install pkg-config"
    )
endif()

pkg_check_modules(TOPLEVEL_ZMQ QUIET libzmq)
if(NOT TOPLEVEL_ZMQ_FOUND)
    message(FATAL_ERROR
        "Missing dependency: libzmq\n"
        "Install development package(s), e.g.:\n"
        "  Ubuntu/Debian: sudo apt install libzmq3-dev\n"
        "  macOS (Homebrew): brew install zeromq"
    )
endif()

pkg_check_modules(TOPLEVEL_LIBUSB QUIET libusb-1.0)
if(NOT TOPLEVEL_LIBUSB_FOUND)
    message(FATAL_ERROR
        "Missing dependency: libusb-1.0\n"
        "Install development package(s), e.g.:\n"
        "  Ubuntu/Debian: sudo apt install libusb-1.0-0-dev\n"
        "  macOS (Homebrew): brew install libusb"
    )
endif()

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/airspyhf_zeromq/cmake/modules ${CMAKE_MODULE_PATH})
find_package(LIBAIRSPYHF QUIET)
if(NOT LIBAIRSPYHF_FOUND)
    message(FATAL_ERROR
        "Missing dependency: libairspyhf\n"
        "Install development package(s), e.g.:\n"
        "  Ubuntu/Debian: sudo apt install libairspyhf-dev\n"
        "  macOS (Homebrew): brew install airspyhf"
    )
endif()

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
add_subdirectory(controller)
add_subdirectory(decimator)
add_subdirectory(airspyhf_zeromq)
