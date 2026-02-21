#include "Connection.h"
#include "timeHelpers.h"
#include "log.h"
#include "MessageParser.h"

#include <optional>

Connection::Connection(MavlinkSystem* mavlink)
	: _mavlink				(mavlink)
{

}

Connection::~Connection()
{

}

bool Connection::start()
{
	if (_open()) {
		_receiveThread 	= std::make_unique<std::thread>(&Connection::_receiveThreadMain, this);
		return true;
	} else {
		return false;
	}
}

void Connection::stop()
{
	_shouldExit = true;

	_close();
	_receiveThread->join();
}

void Connection::_checkForLostHeartbeats()
{
	if (_started && _autopilotFound && _lastReceivedHeartbeatAutopilotMSecs + HEARTBEAT_INTERVAL_MSECS < msecsSinceEpoch()) {
		_heartbeatsLost = true;
		logInfo() << "Heartbeats lost from autopilot";
	}
}

bool Connection::_parseMavlinkBuffer(uint8_t* buffer, size_t cBuffer)
{
	bool prevAutopilotFound = _autopilotFound;

	mavlink_message_t message;
	auto parser = MessageParser(buffer, cBuffer);

	while (parser.parse(&message)) {
		if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			if (message.compid == MAV_COMP_ID_AUTOPILOT1) {
				if (_autopilotFound) {
					if (message.sysid == _sysidAutopilot) {
						if (_heartbeatsLost) {
							_heartbeatsLost = false;
							logInfo() << "Heartbeats regained from autopilot";
						}
						_lastReceivedHeartbeatAutopilotMSecs = msecsSinceEpoch();
					}
				} else {
					_autopilotFound = true;
					_sysidAutopilot = message.sysid;
					_lastReceivedHeartbeatAutopilotMSecs = msecsSinceEpoch();
					logInfo() << "Found autopilot - sysid:" << message.sysid;
				}
			} else {
			    mavlink_heartbeat_t heartbeat;
    			mavlink_msg_heartbeat_decode(&message, &heartbeat);

				if (heartbeat.type == MAV_TYPE_GCS) {
					if (_gcsFound) {
						if (message.sysid == _sysidGcs) {
							_lastReceivedHeartbeatGcsMSecs = msecsSinceEpoch();
						}
					} else if (message.sysid == 255) {
						// We were getting strange GCS connections on other sysids, so we only accept sysid 255 to prevent
						logInfo() << "Found gcs - sysid:" << message.sysid;
						_gcsFound = true;
						_sysidGcs = message.sysid;
						_lastReceivedHeartbeatGcsMSecs = msecsSinceEpoch();
					}

				}
			}
		}

		// Call the message handler callback
		_mavlink->handleMessage(message);

		//_checkForLostHeartbeats();
	}

	return prevAutopilotFound == false && _autopilotFound;	// Is this the first time we are detecting the autopilot?
}

void Connection::_receiveThreadMain()
{
	logInfo() << "Starting receive thread";

	while (!_shouldExit) {
		if (_started) {
			uint8_t buffer[2048];

			ssize_t cBytesReceived =_receiveBytes(buffer, sizeof(buffer)); // Note: this blocks when not receiving any data

			if (cBytesReceived > 0) {
				_parseMavlinkBuffer(buffer, cBytesReceived);
			}
#if 0
			if (_connected && connection_timed_out()) {
				LOG(RED_TEXT "Connection timed out" NORMAL_TEXT);
				_connected = false;
			}
#endif

			// Check if it's time to send a heartbeat. Only send heartbeats if we're still connected to an autopilot
			auto now = msecsSinceEpoch();
			if (_started && _autopilotFound && now > _last_sent_heartbeat_ms + Connection::HEARTBEAT_INTERVAL_MSECS) {
				_mavlink->sendHeartbeat();
				_last_sent_heartbeat_ms = now;
			}
		}
	}

	logInfo() << "Exiting send thread";
}
