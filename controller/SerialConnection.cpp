#include "SerialConnection.h"
#include "MessageParser.h"
#include "MavlinkSystem.h"
#include "log.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>
#include <string.h>

#include <utility>

SerialConnection::SerialConnection(MavlinkSystem* mavlink)
	: Connection(mavlink)
{
	std::string serial              = "serial:";
	std::string serial_flowcontrol  = "serial_flowcontrol:";
	std::string conn                = _mavlink->connectionUrl();

	_flow_control = conn.find(serial_flowcontrol) != std::string::npos;

	std::string& prefix = serial;

	if (_flow_control) {
		prefix = serial_flowcontrol;
	}

	conn.erase(conn.find(prefix), prefix.length());

	size_t index = conn.find(':');
	_serial_node = conn.substr(0, index);
	conn.erase(0, index + 1);
	_baudrate = std::stoi(conn);
}

SerialConnection::~SerialConnection()
{
	// If no one explicitly called stop before, we should at least do it.
	stop();
}

bool SerialConnection::_open()
{
	// open() hangs on macOS or Linux devices(e.g. pocket beagle) unless you give it O_NONBLOCK
	_fd = open(_serial_node.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd == -1) {
		logError() << "_open open failed" << strerror(errno);
		return false;
	}

	// We need to clear the O_NONBLOCK again because we can block while reading
	// as we do it in a separate thread.
	if (fcntl(_fd, F_SETFL, 0) == -1) {
		logError() << "_open fcntl failed" << strerror(errno);
		close(_fd);
		return false;
	}

	struct termios tc;
	bzero(&tc, sizeof(tc));

	if (tcgetattr(_fd, &tc) != 0) {
		logError() << "_open tcgetattr failed" << strerror(errno);
		close(_fd);
		return false;
	}

	tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
	tc.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
	tc.c_cflag |= CS8;

	tc.c_cc[VMIN] = 0; // We are ok with 0 bytes.
	tc.c_cc[VTIME] = 10; // Timeout after 1 second.

	if (_flow_control) {
		tc.c_cflag |= CRTSCTS;
	}

	tc.c_cflag |= CLOCAL; // Without this a write() blocks indefinitely.

	const int baudrate_or_define = define_from_baudrate(_baudrate);

	if (baudrate_or_define == -1) {
		return false;
	}

	if (cfsetispeed(&tc, baudrate_or_define) != 0) {
		logError() << "_open cfsetispeed failed" << strerror(errno);
		close(_fd);
		return false;
	}

	if (cfsetospeed(&tc, baudrate_or_define) != 0) {
		logError() << "_open cfsetospeed failed" << strerror(errno);
		close(_fd);
		return false;
	}

	if (tcsetattr(_fd, TCSANOW, &tc) != 0) {
		logError() << "_open tcsetattr failed" << strerror(errno);
		close(_fd);
		return false;
	}

	_started = true;
	
	return true;
}

void SerialConnection::_close()
{
	close(_fd);
	_fd = -1;
	_started = false;
}

bool SerialConnection::_sendMessage(const mavlink_message_t& message)
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

	int send_len;
	send_len = static_cast<int>(write(_fd, buffer, buffer_len));

	if (send_len != buffer_len) {
		logError() << "_sendMessage write failed" << strerror(errno);
		return false;
	}

	return true;
}

ssize_t SerialConnection::_receiveBytes(uint8_t* buffer, size_t cBuffer)
{
	struct pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int pollrc = poll(fds, 1, 100);

	if (pollrc == 0 || !(fds[0].revents & POLLIN)) {
		return 0;
	} else if (pollrc == -1) {
		logError() << "_receiveBytes poll failed", strerror(errno);
		return -1;
	}

	// We enter here if (fds[0].revents & POLLIN) == true
	auto recv_len = read(_fd, buffer, cBuffer);

	if (recv_len == -1) {
		logError() << "_receiveBytes read failed", strerror(errno);
		return -1;
	}

	return recv_len;
}

#ifndef B460800
	// These are not defined on all platforms, so we define them here if they are missing.
	#define B460800 460800
	#define B500000 500000
	#define B576000 576000
	#define B921600 921600
	#define B1000000 1000000
	#define B1152000 1152000
	#define B1500000 1500000
	#define B2000000 2000000
	#define B2500000 2500000
	#define B3000000 3000000
	#define B3500000 3500000
	#define B4000000 4000000
#endif

int SerialConnection::define_from_baudrate(int baudrate)
{
	switch (baudrate) {
	case 9600:
		return B9600;

	case 19200:
		return B19200;

	case 38400:
		return B38400;

	case 57600:
		return B57600;

	case 115200:
		return B115200;

	case 230400:
		return B230400;

	case 460800:
		return B460800;

	case 500000:
		return B500000;

	case 576000:
		return B576000;

	case 921600:
		return B921600;

	case 1000000:
		return B1000000;

	case 1152000:
		return B1152000;

	case 1500000:
		return B1500000;

	case 2000000:
		return B2000000;

	case 2500000:
		return B2500000;

	case 3000000:
		return B3000000;

	case 3500000:
		return B3500000;

	case 4000000:
		return B4000000;

	default: {
		logError() << "SerialConnection unknown baud rate" << baudrate;
		return -1;
	}
	}
}
