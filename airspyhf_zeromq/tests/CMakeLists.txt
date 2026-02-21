find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_ZMQ_TEST REQUIRED libzmq)

set(TTWF_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/shared)

add_executable(test_zmq_timestamp test_zmq_timestamp.c)
target_include_directories(test_zmq_timestamp PRIVATE ${PC_ZMQ_TEST_INCLUDE_DIRS})
target_include_directories(test_zmq_timestamp PRIVATE ${TTWF_INCLUDE_DIR})
target_compile_options(test_zmq_timestamp PRIVATE ${PC_ZMQ_TEST_CFLAGS_OTHER})
target_link_libraries(test_zmq_timestamp ${PC_ZMQ_TEST_LDFLAGS})

add_dependencies(test_zmq_timestamp airspyhf_zeromq_rx)

add_executable(test_zmq_loss_detection test_zmq_loss_detection.c)
target_include_directories(test_zmq_loss_detection PRIVATE ${PC_ZMQ_TEST_INCLUDE_DIRS})
target_include_directories(test_zmq_loss_detection PRIVATE ${TTWF_INCLUDE_DIR})
target_compile_options(test_zmq_loss_detection PRIVATE ${PC_ZMQ_TEST_CFLAGS_OTHER})
target_link_libraries(test_zmq_loss_detection ${PC_ZMQ_TEST_LDFLAGS})

add_dependencies(test_zmq_loss_detection airspyhf_zeromq_rx)

add_test(
    NAME zmq_timestamp_test
    COMMAND test_zmq_timestamp $<TARGET_FILE:airspyhf_zeromq_rx>
)

add_test(
    NAME zmq_loss_detection_test
    COMMAND test_zmq_loss_detection $<TARGET_FILE:airspyhf_zeromq_rx>
)

set_tests_properties(zmq_timestamp_test PROPERTIES
    SKIP_RETURN_CODE 77
    TIMEOUT 45
)

set_tests_properties(zmq_loss_detection_test PROPERTIES
    SKIP_RETURN_CODE 77
    TIMEOUT 60
)
