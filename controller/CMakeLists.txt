cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CONTROLLER_ZMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(MavlinkTagController2
    main.cpp
    channelizerTuner.cpp channelizerTuner.h
    CommandHandler.cpp CommandHandler.h
    UDPPulseReceiver.cpp UDPPulseReceiver.h
    MonitoredProcess.cpp MonitoredProcess.h
    log.cpp log.h
    formatString.h
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
    PulseHandler.cpp PulseHandler.h
    SimulatorTelemetryPublisher.cpp SimulatorTelemetryPublisher.h
)

target_include_directories(MavlinkTagController2
    PRIVATE
    ${CMAKE_SOURCE_DIR}/tunnel-protocol
    SYSTEM
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(MavlinkTagController2
    PRIVATE
    ${Boost_LIBRARIES}
    mavlink
    PkgConfig::CONTROLLER_ZMQ
)
