#include "MavlinkSystem.h"
#include "log.h"
#include "UdpConnection.h"
#include "SerialConnection.h"
#include "TunnelProtocol.h"

#include <mutex>
#include <fstream>
#include <functional>
#include <sys/time.h>

static MavlinkSystem* _instance = nullptr;

MavlinkSystem::MavlinkSystem()
	: _outgoingMessageQueue	(this)
	, _telemetry			(this)
{
	// Force all output to Mavlink V2
	mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(0);
	mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;	
}

MavlinkSystem::~MavlinkSystem()
{
	stop();
}

MavlinkSystem* MavlinkSystem::instance()
{
	if (_instance == nullptr) {
		_instance = new MavlinkSystem;
	}
	return _instance;
}

void MavlinkSystem::init(const std::string& connectionUrl)
{
	_connectionUrl = connectionUrl;
}

bool MavlinkSystem::start()
{
	if (_connectionUrl.find("serial:") != std::string::npos ||
	    _connectionUrl.find("serial_flowcontrol:") != std::string::npos) {

		_connection = std::make_unique<SerialConnection>(this);

	} else if (_connectionUrl.find("udp:") != std::string::npos) {

		_connection = std::make_unique<UdpConnection>(this);

	} else {
		logError() << "Invalid connection string:" << _connectionUrl.c_str();
	}

    subscribeToMessage(MAVLINK_MSG_ID_SYSTEM_TIME, std::bind(&MavlinkSystem::_handleSystemTime, this, std::placeholders::_1));

	return _connection->start();
}

void MavlinkSystem::stop()
{
	// Waits for connection threads to join
	if (_connection.get()) _connection->stop();
}

bool MavlinkSystem::connected()
{
	return _connection.get() && _connection->connected();
}

void MavlinkSystem::handleMessage(const mavlink_message_t& message)
{
	std::scoped_lock<std::mutex> lock(_subscriptions_mutex);

	if (_message_subscriptions.contains(message.msgid)) {
		_message_subscriptions[message.msgid](message);
	}
}

void MavlinkSystem::subscribeToMessage(uint16_t message_id, const MessageCallback& callback)
{
	std::scoped_lock<std::mutex> lock(_subscriptions_mutex);

	if (_message_subscriptions.find(message_id) == _message_subscriptions.end()) {
		logInfo() << "subscribeToMessage" << message_id;
		_message_subscriptions.emplace(message_id, callback);
	} else {
		logError() << "MavlinkSystem::subscribe_to_message failed, callback already registered";
	}
}

void MavlinkSystem::sendMessage(const mavlink_message_t& message)
{
	_outgoingMessageQueue.addMessage(message);
}

void MavlinkSystem::sendHeartbeat()
{
    if (!ourSystemId().has_value()) {
        logError() << "Called before autopilot discovered";
        return;
    }

	mavlink_heartbeat_t heartbeat;

	memset(&heartbeat, 0, sizeof(heartbeat));
	heartbeat.type 			= MAV_TYPE_ONBOARD_CONTROLLER;
	heartbeat.autopilot 	= MAV_AUTOPILOT_INVALID;
	heartbeat.system_status = MAV_STATE_ACTIVE;

	mavlink_message_t message;
	mavlink_msg_heartbeat_encode(ourSystemId().value(), ourComponentId(), &message, &heartbeat);

	sendMessage(message);
}

void MavlinkSystem::sendStatusText(std::string& text, MAV_SEVERITY severity)
{
    if (!gcsSystemId().has_value()) {
        logError() << "Called before gcs discovered";
        return;
    }

	logInfo() << "statustext:" << text;

	mavlink_statustext_t statustext;
	
	memset(&statustext, 0, sizeof(statustext));
	statustext.severity = severity;

	snprintf(statustext.text, sizeof(statustext.text), "%s", text.c_str());

	mavlink_message_t message;
	mavlink_msg_statustext_encode(ourSystemId().value(), ourComponentId(), &message, &statustext);

	sendMessage(message);
}

void MavlinkSystem::sendTunnelMessage(void* tunnelPayload, size_t tunnelPayloadSize)
{
    if (!gcsSystemId().has_value()) {
        logError() << "Called before gcs discovered";
        return;
    }

    mavlink_message_t   message;
    mavlink_tunnel_t    tunnel;

    memset(&tunnel, 0, sizeof(tunnel));

    tunnel.target_system    = gcsSystemId().value();
    tunnel.target_component = 0;
    tunnel.payload_type     = MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN;
    tunnel.payload_length   = tunnelPayloadSize;

    memcpy(tunnel.payload, tunnelPayload, tunnelPayloadSize);

    mavlink_msg_tunnel_encode(
        ourSystemId().value(),
        ourComponentId(),
        &message,
        &tunnel);

    sendMessage(message);
}

std::optional<uint8_t> MavlinkSystem::ourSystemId() const 
{
	if (_connection.get()) {
		return _connection->autopilotSystemId();
	} else {
		return std::nullopt;
	}
}

std::optional<uint8_t> MavlinkSystem::gcsSystemId() const 
{
	if (_connection.get()) {
		return _connection->gcsSystemId();
	} else {
		return std::nullopt;
	}
}

float MavlinkSystem::_cpuTemp()
{
    std::string         fileName = "/sys/class/thermal/thermal_zone0/temp";
    std::ifstream       stream;
    std::stringstream   buffer;
    float               temp = 0.0;

    stream.open(fileName);
    if (stream.is_open()) {
        buffer << stream.rdbuf();
        stream.close();

        temp = std::stof(buffer.str());     // convert string to float
        temp = temp / 1000;                 // convert float value to degree
        temp = roundf(temp * 100) / 100;    // round decimal to nearest
    } else {
		static bool notified = false;
		if (!notified) {
			notified = true;
	        logError() << "Failed to open CPU temperature file";
		}
		temp = std::nanf("");
    }

	return temp;
}

void MavlinkSystem::startTunnelHeartbeatSender()
{
    std::thread heartbeatSenderThread([this]() {
        int cpuWaitCount = 0;

        while (true) {
            TunnelProtocol::Heartbeat_t heartbeat;

            heartbeat.header.command    = COMMAND_ID_HEARTBEAT;
            heartbeat.system_id         = HEARTBEAT_SYSTEM_ID_MAVLINKCONTROLLER;
			heartbeat.status			= _heartbeatStatus;
			heartbeat.cpu_temp_c		= _cpuTemp();

            sendTunnelMessage(&heartbeat, sizeof(heartbeat));

            if (cpuWaitCount-- <= 0) {
                cpuWaitCount = 30;
        		logInfo() << "CPU Temperature: " << _cpuTemp() << "°C";
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
    heartbeatSenderThread.detach();
}

void MavlinkSystem::_sendMessageOnConnection(const mavlink_message_t& message)
{
	_connection->_sendMessage(message);
}

void MavlinkSystem::_handleSystemTime(const mavlink_message_t& message)
{
	static bool timeSetFailed = false;
    mavlink_system_time_t system_time;

    mavlink_msg_system_time_decode(&message, &system_time);

	auto vehicleEpochTimeSeconds = system_time.time_unix_usec / 1000000;
	auto vehicleEpochTimeT = static_cast<std::time_t>(vehicleEpochTimeSeconds);

    auto rpiEpochTimeT = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

	if (static_cast<uint64_t>(vehicleEpochTimeT) != static_cast<uint64_t>(rpiEpochTimeT)) {
		struct timeval tv;

		logInfo() << "Synchronizing rPi time to vehicle time";
		tv.tv_sec = static_cast<uint64_t>(vehicleEpochTimeT);
		tv.tv_usec = 0;
		if (!timeSetFailed && settimeofday(&tv, NULL) != 0) {
			timeSetFailed = true;
			logError() << "Failed to set rPi time of day: " << strerror(errno);
		}
	}
}
