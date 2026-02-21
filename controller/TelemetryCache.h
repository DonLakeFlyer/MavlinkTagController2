#pragma once

#include <chrono>
#include <list>
#include <mutex>

#include <mavlink.h>

#include "Telemetry.h"

class MavlinkSystem;

class TelemetryCache
{
public:
	typedef struct {
		double					timeInSeconds;
		Telemetry::Position_t 	position;
		Telemetry::EulerAngle_t attitudeEuler;
	} TelemetryCacheEntry_t;

	TelemetryCache(MavlinkSystem* mavlink);
	~TelemetryCache();

	TelemetryCacheEntry_t telemetryForTime(double timeInSeconds);

private:
	void _pruneTelemetryCache();

	MavlinkSystem* 						_mavlink;
	std::list<TelemetryCacheEntry_t> 	_telemetryCache;
	std::mutex							_telemetryCacheMutex;
};