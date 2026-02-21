#include "UdpConnection.h"
#include "MessageParser.h"
#include "MavlinkSystem.h"
#include "log.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <errno.h>
#include <string.h>

// TODO: overload constructor to pass in connection target -- target.sysid and target.compid (instead of 1/1 for autopilot)
UdpConnection::UdpConnection(MavlinkSystem* mavlink)
	: Connection(mavlink)
{
	std::string udp = "udp:";
	std::string conn = _mavlink->connectionUrl();

	conn.erase(conn.find(udp), udp.length());

	size_t index = conn.find(':');
	_our_ip = conn.substr(0, index);
	conn.erase(0, index + 1);
	_our_port = std::stoi(conn);
}

bool UdpConnection::_open()
{
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_socket_fd < 0) {
		logError() << "_open - socket error" << strerror(errno);
		return false;
	}

	struct sockaddr_in addr = {};

	addr.sin_family = AF_INET;

	inet_pton(AF_INET, _our_ip.c_str(), &(addr.sin_addr));

	addr.sin_port = htons(_our_port);

	if (bind(_socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
		logError() << "_open - bind failed" << strerror(errno);
		return false;
	}

	_started = true;

	return true;
}

void UdpConnection::_close()
{
	shutdown(_socket_fd, SHUT_RDWR);
	close(_socket_fd);
	_started = false;
}

bool UdpConnection::_sendMessage(const mavlink_message_t& message)
{
	struct sockaddr_in dest_addr {};
	dest_addr.sin_family = AF_INET;

	inet_pton(AF_INET, _remote_ip.c_str(), &dest_addr.sin_addr.s_addr);
	dest_addr.sin_port = htons(_remote_port);

	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

	const auto send_len = sendto(_socket_fd, reinterpret_cast<char*>(buffer), buffer_len,
				     0, reinterpret_cast<const sockaddr*>(&dest_addr), sizeof(dest_addr));

	return send_len == buffer_len;
}

ssize_t UdpConnection::_receiveBytes(uint8_t* buffer, size_t cBuffer)
{
	struct sockaddr_in src_addr = {};
	socklen_t src_addr_len = sizeof(src_addr);

	// NOTE: This function blocks -- thus during destruction we call shutdown/close on the socket before joining the thread
	// TODO: this isn't actually returning if there's no data coming in
	// We need to find a way to signal to the thread to unblock from recvfrom
	auto cBytesReceived =  recvfrom(
								_socket_fd,
								buffer,
								cBuffer,
								0,
								reinterpret_cast<struct sockaddr*>(&src_addr),
								&src_addr_len);

	_remote_ip = inet_ntoa(src_addr.sin_addr);
	_remote_port = ntohs(src_addr.sin_port);

	return cBytesReceived;
}
