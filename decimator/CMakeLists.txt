cmake_minimum_required(VERSION 3.16)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(airspyhf_decimator
    src/main.cpp
)

target_include_directories(airspyhf_decimator PRIVATE
    ${CMAKE_SOURCE_DIR}/shared
)

target_link_libraries(airspyhf_decimator PRIVATE PkgConfig::ZeroMQ)



if(BUILD_TESTING)
    add_executable(airspyhf_decimator_tests
        tests/test_main.cpp
    )

    target_include_directories(airspyhf_decimator_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/shared
    )

    target_link_libraries(airspyhf_decimator_tests PRIVATE PkgConfig::ZeroMQ)

    add_test(NAME airspyhf_decimator_tests COMMAND airspyhf_decimator_tests)
endif()
