set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CONTROLLER_ZMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(MavlinkTagController2
    main.cpp
    CollectionCoordinator.cpp CollectionCoordinator.h
    DetectionCoordinator.cpp DetectionCoordinator.h
    OperationProgress.cpp OperationProgress.h
    RequestCache.cpp RequestCache.h
    TunnelCommandDispatcher.cpp TunnelCommandDispatcher.h
    channelizerTuner.cpp channelizerTuner.h
    CommandHandler.cpp CommandHandler.h
    PythonPulseMapper.cpp PythonPulseMapper.h
    UDPPulseReceiver.cpp UDPPulseReceiver.h
    MonitoredProcess.cpp MonitoredProcess.h
    log.cpp log.h
    formatString.h
    platformHelpers.h
    TagDatabase.cpp TagDatabase.h
    TagUploadCoordinator.cpp TagUploadCoordinator.h
    TelemetryCache.cpp TelemetryCache.h
    MavlinkOutgoingMessageQueue.cpp MavlinkOutgoingMessageQueue.h
    Connection.cpp Connection.h
    MavlinkSystem.cpp MavlinkSystem.h
    MessageParser.h
    SerialConnection.cpp SerialConnection.h
    UdpConnection.cpp UdpConnection.h
    Telemetry.cpp Telemetry.h
    timeHelpers.cpp timeHelpers.h
    LogFileManager.cpp LogFileManager.h
    MavlinkFtpServer.cpp MavlinkFtpServer.h
    AntennaPattern.cpp AntennaPattern.h
    BearingCalculator.cpp BearingCalculator.h
    SimulatorTelemetryPublisher.cpp SimulatorTelemetryPublisher.h
)

target_include_directories(MavlinkTagController2
    PRIVATE
    ${CMAKE_SOURCE_DIR}/shared
    SYSTEM
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(MavlinkTagController2
    PRIVATE
    ${Boost_LIBRARIES}
    mavlink
    tunnel_protocol
    PkgConfig::CONTROLLER_ZMQ
)

if(BUILD_TESTING)
    # Offline tuning aid for AntennaPattern::confidenceFloor; not part of the tests.
    add_executable(confidence_floor_montecarlo
        tools/confidence_floor_montecarlo.cpp
        AntennaPattern.cpp AntennaPattern.h
        BearingCalculator.cpp BearingCalculator.h
    )
    target_include_directories(confidence_floor_montecarlo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(confidence_floor_montecarlo PRIVATE tunnel_protocol)

    add_executable(test_collection_protocol
        tests/test_collection_protocol.cpp
    )
    target_link_libraries(test_collection_protocol PRIVATE tunnel_protocol)
    add_test(NAME test_collection_protocol COMMAND test_collection_protocol)

    add_executable(test_collection_coordinator
        tests/test_collection_coordinator.cpp
        CollectionCoordinator.cpp CollectionCoordinator.h
    )
    target_include_directories(test_collection_coordinator PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(test_collection_coordinator PRIVATE tunnel_protocol)
    add_test(NAME test_collection_coordinator COMMAND test_collection_coordinator)

    add_executable(test_detector_protocol
        tests/test_detector_protocol.cpp
    )
    target_include_directories(test_detector_protocol PRIVATE
        ${CMAKE_SOURCE_DIR}/shared
    )
    add_test(NAME test_detector_protocol COMMAND test_detector_protocol)

    add_executable(test_bearing_calculator
        tests/test_bearing_calculator.cpp
        AntennaPattern.cpp AntennaPattern.h
        BearingCalculator.cpp BearingCalculator.h
    )
    target_include_directories(test_bearing_calculator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_bearing_calculator PRIVATE tunnel_protocol)
    add_test(NAME test_bearing_calculator COMMAND test_bearing_calculator)

    add_executable(test_python_pulse_mapper
        tests/test_python_pulse_mapper.cpp
        PythonPulseMapper.cpp PythonPulseMapper.h
    )
    target_include_directories(test_python_pulse_mapper PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/shared
    )
    target_link_libraries(test_python_pulse_mapper PRIVATE mavlink tunnel_protocol)
    add_test(NAME test_python_pulse_mapper COMMAND test_python_pulse_mapper)

    add_executable(test_tag_database
        tests/test_tag_database.cpp
    )
    target_include_directories(test_tag_database PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_tag_database PRIVATE mavlink tunnel_protocol)
    add_test(NAME test_tag_database COMMAND test_tag_database)

    add_executable(test_tag_upload_coordinator
        tests/test_tag_upload_coordinator.cpp
        TagUploadCoordinator.cpp TagUploadCoordinator.h
    )
    target_include_directories(test_tag_upload_coordinator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_tag_upload_coordinator PRIVATE mavlink tunnel_protocol)
    add_test(NAME test_tag_upload_coordinator COMMAND test_tag_upload_coordinator)

    add_executable(test_request_cache
        tests/test_request_cache.cpp
        RequestCache.cpp RequestCache.h
    )
    target_include_directories(test_request_cache PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME test_request_cache COMMAND test_request_cache)

    add_executable(test_detection_coordinator
        tests/test_detection_coordinator.cpp
        DetectionCoordinator.cpp DetectionCoordinator.h
    )
    target_include_directories(test_detection_coordinator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_detection_coordinator PRIVATE tunnel_protocol)
    add_test(NAME test_detection_coordinator COMMAND test_detection_coordinator)

    # Whole command/ack path with lost-ACK retries, no MAVLink or processes.
    add_executable(test_command_retry
        tests/test_command_retry.cpp
        TunnelCommandDispatcher.cpp TunnelCommandDispatcher.h
        DetectionCoordinator.cpp DetectionCoordinator.h
        OperationProgress.cpp OperationProgress.h
        RequestCache.cpp RequestCache.h
        TagUploadCoordinator.cpp TagUploadCoordinator.h
    )
    target_include_directories(test_command_retry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_command_retry PRIVATE mavlink tunnel_protocol)
    add_test(NAME test_command_retry COMMAND test_command_retry)

    add_executable(test_operation_progress
        tests/test_operation_progress.cpp
        OperationProgress.cpp OperationProgress.h
    )
    target_include_directories(test_operation_progress PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_operation_progress PRIVATE tunnel_protocol)
    add_test(NAME test_operation_progress COMMAND test_operation_progress)
endif()
