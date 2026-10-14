#pragma once

#include <chrono>
#include <optional>
#include <mutex>

#include <mavlink.h>

class MavlinkSystem;

class Telemetry
{
public:
	typedef struct {
		double 	latitude;
		double 	longitude;
		double 	relativeAltitude;
	} Position_t;

	typedef struct {
		float rollDegrees;
		float pitchDegrees;
		float yawDegrees;
	} EulerAngle_t;

	typedef struct {
		float w;
		float x;
		float y;
		float z;
	} Quaternion_t;

	Telemetry(MavlinkSystem* mavlink);

	std::optional<Position_t>		lastPosition();			// thread safe
	std::optional<EulerAngle_t> 	lastAttitudeEuler();	// thread safe
    // As above, but nullopt when the last message is older than maxAgeSeconds (thread safe)
    std::optional<Position_t>       freshPosition(double maxAgeSeconds);
    std::optional<EulerAngle_t>     freshAttitudeEuler(double maxAgeSeconds);

private:
	void 			_positionCallback			(const mavlink_message_t& message);
	void 			_attitudeCallback			(const mavlink_message_t& message);
	float 			_toDegFromRad				(float rad);
	EulerAngle_t 	_toEulerAngleFromQuaternion	(Quaternion_t quaternion);
	float			_radiansToDegrees			(float radians);

	MavlinkSystem* 					_mavlink;
	std::optional<Position_t>		_lastPosition;
	std::optional<EulerAngle_t> 	_lastAttitudeEuler;
    std::chrono::steady_clock::time_point _lastPositionTime;
    std::chrono::steady_clock::time_point _lastAttitudeTime;
	std::mutex						_accessMutex;
};
