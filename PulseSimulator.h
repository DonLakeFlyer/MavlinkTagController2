#pragma once

#include <mavlink.h>

#include "ThreadSafeQueue.h"
#include "MavlinkSystem.h"

#include <string>
#include <thread>
#include <optional>

class PulseHandler;

class PulseSimulator
{
public:
	PulseSimulator(PulseHandler* pulseHandler, MavlinkSystem* mavlink, uint32_t antennaOffset);

	void startSendingTagPulses	(uint32_t frequencyHz) { _frequencyHz = frequencyHz; _sendTagPulses = true; }
	void stopSendingTagPulses	(void) { _sendTagPulses = false; }

private:
	double _snrFromYaw	(double vehicleYawDegrees);
	double _normalizeYaw(double yaw);

	PulseHandler* 	_pulseHandler	= nullptr;
	MavlinkSystem* 	_mavlink 		= nullptr;
	uint32_t 		_antennaOffset 	= 0;
	uint32_t		_frequencyHz 	= 146000000;
	bool 			_sendTagPulses 	= false;
};
