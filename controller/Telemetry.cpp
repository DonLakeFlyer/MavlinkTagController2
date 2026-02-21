#include "Telemetry.h"
#include "MavlinkSystem.h"
#include "log.h"

#include <functional>
#include <chrono>
#include <cmath>

Telemetry::Telemetry(MavlinkSystem* mavlink)
    : _mavlink(mavlink)
{
	_mavlink->subscribeToMessage(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,    std::bind(&Telemetry::_positionCallback, this, std::placeholders::_1));
	_mavlink->subscribeToMessage(MAVLINK_MSG_ID_ATTITUDE,               std::bind(&Telemetry::_attitudeCallback, this, std::placeholders::_1));
}

void Telemetry::_positionCallback(const mavlink_message_t& message)
{
    mavlink_global_position_int_t globalPositionInt;
    mavlink_msg_global_position_int_decode(&message, &globalPositionInt);

    // ArduPilot sends bogus GLOBAL_POSITION_INT messages with lat/lat 0/0 even when it has no gps signal
    // Apparently, this is in order to transport relative altitude information.
    if (globalPositionInt.lat == 0 && globalPositionInt.lon == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(_accessMutex);

    Position_t lastPosition;
    lastPosition.latitude           = globalPositionInt.lat  / (double)1E7;
    lastPosition.longitude          = globalPositionInt.lon  / (double)1E7;
    lastPosition.relativeAltitude   = globalPositionInt.relative_alt * 1e-3f;

    _lastPosition = lastPosition;
}

float Telemetry::_toDegFromRad(float rad)
{
    return 180.0f / static_cast<float>(M_PI) * rad;
}

Telemetry::EulerAngle_t Telemetry::_toEulerAngleFromQuaternion(Telemetry::Quaternion_t quaternion)
{
    auto& q = quaternion;

    EulerAngle_t eulerAngle;
    eulerAngle.rollDegrees = _toDegFromRad(
        atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)));
    eulerAngle.pitchDegrees = _toDegFromRad(asinf(2.0f * (q.w * q.y - q.z * q.x)));
    eulerAngle.yawDegrees = _toDegFromRad(
        atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)));

    return eulerAngle;
}

float Telemetry::_radiansToDegrees(float radians)
{
    return radians * (180.0f / static_cast<float>(M_PI));
}

void Telemetry::_attitudeCallback(const mavlink_message_t& message)
{
    // only accept the attitude message from the vehicle's flight controller
    if (message.sysid != _mavlink->ourSystemId() || message.compid != MAV_COMP_ID_AUTOPILOT1) {
        return;
    }

    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&message, &attitude);

    std::lock_guard<std::mutex> lock(_accessMutex);

    EulerAngle_t lastAttitudeEuler;
    lastAttitudeEuler.rollDegrees  = _radiansToDegrees(attitude.roll);
    lastAttitudeEuler.pitchDegrees = _radiansToDegrees(attitude.pitch);
    lastAttitudeEuler.yawDegrees   = _radiansToDegrees(attitude.yaw);

    _lastAttitudeEuler = lastAttitudeEuler;
}

std::optional<Telemetry::Position_t> Telemetry::lastPosition()
{
    std::lock_guard<std::mutex> lock(_accessMutex);
    return _lastPosition;
}

std::optional<Telemetry::EulerAngle_t> Telemetry::lastAttitudeEuler()
{
    std::lock_guard<std::mutex> lock(_accessMutex);
    return _lastAttitudeEuler;
}
