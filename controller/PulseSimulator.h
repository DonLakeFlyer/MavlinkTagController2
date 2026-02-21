#pragma once

#include <mavlink.h>

#include "ThreadSafeQueue.h"
#include "MavlinkSystem.h"
#include "Telemetry.h"

#include <string>
#include <thread>
#include <optional>

class PulseHandler;
class TelemetryCache;

class PulseSimulator
{
public:
	PulseSimulator(PulseHandler* pulseHandler, MavlinkSystem* mavlink, TelemetryCache* telemetryCache, int32_t antennaOffset);

	void startSendingTagPulses	(uint32_t frequencyHz) { _frequencyHz = frequencyHz; _sendTagPulses = true; }
	void stopSendingTagPulses	(void) { _sendTagPulses = false; }

private:
	double _calcSNR(const Telemetry::Position_t& vehiclePosition, const Telemetry::Position_t& tagPosition, const Telemetry::EulerAngle_t& vehicleAttitudeEuler);
	double _normalizeYaw(double yaw);
	double _toRadians(double degree);
	double _toDegrees(double radian);
	double _haversineDistanceKilometers(double lat1, double lon1, double lat2, double lon2);
	double _calculateAzimuth(double lat1, double lon1, double lat2, double lon2);

	PulseHandler* 	_pulseHandler	= nullptr;
	MavlinkSystem* 	_mavlink 		= nullptr;
	TelemetryCache* _telemetryCache	= nullptr;
	int32_t 		_antennaOffset 	= 0;
	uint32_t		_frequencyHz 	= 146000000;
	bool 			_sendTagPulses 	= false;
	std::optional<Telemetry::Position_t> _tagPosition;
};
