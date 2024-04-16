cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

project(MavlinkTagController2)

add_definitions("-Wall -Wextra -Wno-address-of-packed-member")

set(Boost_USE_MULTITHREADED ON) 
find_package( Boost REQUIRED COMPONENTS system filesystem )

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
    uavrt_interfaces/include/uavrt_interfaces/TunnelProtocol.h
    Connection.cpp Connection.h
    MavlinkSystem.cpp MavlinkSystem.h
    MessageParser.h
    SerialConnection.cpp SerialConnection.h
    UdpConnection.cpp UdpConnection.h
    Telemetry.cpp Telemetry.h
    PulseSimulator.cpp PulseSimulator.h
    timeHelpers.cpp timeHelpers.h
    LogFileManager.cpp LogFileManager.h
    MavlinkFtpServer.cpp MavlinkFtpServer.h
)

target_include_directories(MavlinkTagController2
    PRIVATE
    uavrt_interfaces/include/uavrt_interfaces
    mavlink/v2/common
)

target_link_libraries(MavlinkTagController2
    PRIVATE
    ${Boost_LIBRARIES}
)
