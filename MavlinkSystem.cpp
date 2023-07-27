#include "MavlinkSystem.h"
#include "log.h"
#include "UdpConnection.h"
#include "SerialConnection.h"
#include "TunnelProtocol.h"

#include <mutex>

MavlinkSystem::MavlinkSystem(const std::string& connectionUrl)
	: _connectionUrl		(connectionUrl)
	, _outgoingMessageQueue	(this)
{
	// Force all output to Mavlink V2
	mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(0);
	mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;	
}

MavlinkSystem::~MavlinkSystem()
{
	stop();
}

bool MavlinkSystem::start()
{
	if (_connectionUrl.find("serial:") != std::string::npos ||
	    _connectionUrl.find("serial_flowcontrol:") != std::string::npos) {

		_connection = std::make_unique<SerialConnection>(this);

	} else if (_connectionUrl.find("udp:") != std::string::npos) {

		_connection = std::make_unique<UdpConnection>(this);

	} else {
		logError() << "Invalid connection string:", _connectionUrl.c_str();
	}

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

void MavlinkSystem::sendStatusText(std::string&& text, MAV_SEVERITY severity)
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

void MavlinkSystem::startTunnelHeartbeatSender()
{
    std::thread heartbeatSenderThread([this]() {
        while (true) {
            TunnelProtocol::Heartbeat_t heartbeat;

            heartbeat.header.command    = COMMAND_ID_HEARTBEAT;
            heartbeat.system_id         = HEARTBEAT_SYSTEM_MAVLINKCONTROLLER;

            sendTunnelMessage(&heartbeat, sizeof(heartbeat));

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
    heartbeatSenderThread.detach();
}

void MavlinkSystem::_sendMessageOnConnection(const mavlink_message_t& message)
{
	_connection->_sendMessage(message);
}
