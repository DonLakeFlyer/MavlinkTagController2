#pragma once

#include <mavlink.h>

#include "ThreadSafeQueue.h"
#include "MavlinkSystem.h"

#include <string>
#include <thread>
#include <optional>

class Connection
{
public:
	Connection(MavlinkSystem* mavlink);
	virtual ~Connection();

	bool start					();
	void stop					();
	bool connected				() const { return _autopilotFound; };

	std::optional<uint8_t> autopilotSystemId	() const { return _sysidAutopilot; };
	std::optional<uint8_t> gcsSystemId			() const { return _sysidGcs; };

	virtual bool _sendMessage(const mavlink_message_t& message) = 0;

	static constexpr uint64_t HEARTBEAT_INTERVAL_MSECS = 1000; // 1Hz

protected:
	virtual bool 	_open			() = 0;
	virtual void 	_close			() = 0;
	virtual ssize_t _receiveBytes	(uint8_t* buffer, size_t cBuffer) = 0;	// Blocking call

	bool _parseMavlinkBuffer(uint8_t* buffer, size_t cBuffer);
	void _checkForLostHeartbeats();
	void _receiveThreadMain();

	std::atomic_bool	_shouldExit {};
	std::atomic_bool	_started {};
	std::atomic_bool 	_autopilotFound {};
	std::atomic_bool 	_gcsFound {};
	std::atomic_bool	_heartbeatsLost {};

	std::optional<uint8_t> 	_sysidAutopilot;
	std::optional<uint8_t> 	_sysidGcs;
	uint64_t 				_lastReceivedHeartbeatAutopilotMSecs 	{};
	uint64_t 				_lastReceivedHeartbeatGcsMSecs 			{};
	uint64_t 				_last_sent_heartbeat_ms 				{};

	std::unique_ptr<std::thread> _receiveThread {};

	MavlinkSystem* _mavlink {}; 
};
