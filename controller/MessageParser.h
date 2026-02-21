#pragma once

#include <mavlink.h>

class MessageParser
{
public:
	MessageParser(uint8_t* buffer, size_t bufferLen)
		: _buffer	(buffer)
		, _bufferLen(bufferLen)
	{}

	// Parses a single mavlink message from the data buffer.
	// Note that one datagram can contain multiple mavlink messages.
	// It is OK if a message is fragmented because mavlink_parse_char places bytes into an internal buffer.
	bool parse(mavlink_message_t* message)
	{
		for (size_t i = 0; i < _bufferLen; ++i) {
			mavlink_status_t status;

			if (mavlink_parse_char(0, _buffer[i], message, &status)) {
				// Move the pointer to the data forward by the amount parsed.
				_buffer += (i + 1);
				// And decrease the length, so we don't overshoot in the next round.
				_bufferLen -= (i + 1);
				// We have parsed one message, let's return so it can be handled.
				return true;
			}
		}

		// No more messages.
		return false;
	}
private:
	uint8_t*	_buffer {};
	size_t 		_bufferLen {};
};