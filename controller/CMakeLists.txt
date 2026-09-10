set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CONTROLLER_ZMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(MavlinkTagController2
    main.cpp
    CollectionCoordinator.cpp CollectionCoordinator.h
    channelizerTuner.cpp channelizerTuner.h
    CommandHandler.cpp CommandHandler.h
    PythonPulseMapper.cpp PythonPulseMapper.h
    UDPPulseReceiver.cpp UDPPulseReceiver.h
    MonitoredProcess.cpp MonitoredProcess.h
    log.cpp log.h
    formatString.h
    platformHelpers.h
    TagDatabase.cpp TagDatabase.h
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
endif()
