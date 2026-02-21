#pragma once

#include <atomic>
#include <functional>
#include <queue>
#include <mutex>
#include <memory>
#include <unordered_map>
#include <ctime>

#include "ThreadSafeQueue.h"
#include "MavlinkOutgoingMessageQueue.h"
#include "Telemetry.h"
#include "TunnelProtocol.h"

#include <mavlink.h>

#include <optional>

using MessageCallback = std::function<void(const mavlink_message_t&)>;

class Connection;

class MavlinkSystem
{
public:
	~MavlinkSystem();

	static MavlinkSystem* instance();
	void init(const std::string& connectionUrl);

	bool start();
	void stop();

	std::optional<uint8_t> 		ourSystemId					() const;
	uint8_t 					ourComponentId				() const { return MAV_COMP_ID_ONBOARD_COMPUTER; }
	std::optional<uint8_t> 		gcsSystemId					() const;
	const std::string& 			connectionUrl				() const { return _connectionUrl; }
	void 						subscribeToMessage			(uint16_t message_id, const MessageCallback& callback);
	void 						handleMessage				(const mavlink_message_t& message);
	void 						startTunnelHeartbeatSender	();
	bool 						connected					();
	void 						sendHeartbeat				();
	void 						sendStatusText				(std::string& message, MAV_SEVERITY severity = MAV_SEVERITY_INFO);
	void 						sendStatusText				(const char* message, MAV_SEVERITY severity = MAV_SEVERITY_INFO) { std::string str(message); sendStatusText(str, severity); }
	void 						sendTunnelMessage			(void* tunnelPayload, size_t tunnelPayloadSize);
	void 						sendMessage					(const mavlink_message_t& message);
	Telemetry& 					telemetry					() { return _telemetry; }
	uint16_t 					heartbeatStatus				() const { return _heartbeatStatus; }
	void						setHeartbeatStatus			(uint16_t heartbeatStatus) { _heartbeatStatus = heartbeatStatus; }

private:
	MavlinkSystem();

	void 	_sendMessageOnConnection(const mavlink_message_t& message);
    float 	_cpuTemp				();
    void 	_handleSystemTime   	(const mavlink_message_t& message);

	std::unordered_map<uint16_t, MessageCallback> _message_subscriptions {}; // Mavlink message ID --> callback(mavlink_message_t)

	std::string 				_connectionUrl {};
	MavlinkOutgoingMessageQueue _outgoingMessageQueue;
	std::unique_ptr<Connection> _connection {};
	std::mutex 					_subscriptions_mutex {};
	Telemetry 					_telemetry;
	uint16_t					_heartbeatStatus { HEARTBEAT_STATUS_IDLE };

	friend class MavlinkOutgoingMessageQueue;
};
