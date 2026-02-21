#pragma once

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <functional>

#include <time.h>

#include "Connection.h"
#include "timeHelpers.h"

class MavlinkSystem;

class UdpConnection : public Connection
{
public:
	UdpConnection(MavlinkSystem* mavlink);

	// Non-copyable
	UdpConnection(const UdpConnection&) = delete;
	const UdpConnection& operator=(const UdpConnection&) = delete;

protected:
	// Connection overrides
	bool 	_open			() override;
	void 	_close			() override;
	ssize_t _receiveBytes	(uint8_t* buffer, size_t cBuffer) override;
	bool 	_sendMessage	(const mavlink_message_t& message) override;

	// Our IP and port
	std::string _our_ip {};
	int _our_port {};

	// Autopilot IP and port
	std::string _remote_ip {};
	int _remote_port {};

	// Connection
	int _socket_fd {-1};
	std::unique_ptr<std::thread> _recv_thread {};
	std::unique_ptr<std::thread> _send_thread {};
	std::atomic_bool _should_exit {false};
	char _receive_buffer[2048] {}; // Enough for MTU 1500 bytes.

	// Mavlink internal data
	char* _datagram {};
	unsigned _datagram_len {};
};
