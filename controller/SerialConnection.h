#pragma once

#include <mutex>
#include <memory>
#include <atomic>
#include <thread>

#include "Connection.h"
#include "timeHelpers.h"

class Mavlink;

class SerialConnection : public Connection
{

public:
	SerialConnection(MavlinkSystem* mavlink);
	~SerialConnection();

	// Non-copyable
	SerialConnection(const SerialConnection&) = delete;
	const SerialConnection& operator=(const SerialConnection&) = delete;

protected:
	// Connection overrides
	bool 	_open			() override;
	void 	_close			() override;
	ssize_t _receiveBytes	(uint8_t* buffer, size_t cBuffer) override;
	bool 	_sendMessage	(const mavlink_message_t& message) override;

	static int define_from_baudrate(int baudrate);

	std::string _serial_node {};
	int         _baudrate {};
	bool        _flow_control {};

	std::mutex _mutex = {};
	int _fd = -1;

	std::unique_ptr<std::thread> _recv_thread{};
	std::atomic_bool _should_exit{false};

	Mavlink* _parent {};
};
