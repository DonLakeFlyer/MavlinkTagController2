cmake_minimum_required(VERSION 3.12)
project(airspyhf_zeromq C)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

message(STATUS "Using system libairspyhf")
add_subdirectory(tools)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
