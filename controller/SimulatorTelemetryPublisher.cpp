#include "SimulatorTelemetryPublisher.h"

#include "MavlinkSystem.h"
#include "Telemetry.h"
#include "log.h"
#include "timeHelpers.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#include <zmq.h>

SimulatorTelemetryPublisher::SimulatorTelemetryPublisher(MavlinkSystem* mavlink, const std::string& endpoint, int publishPeriodMs)
	: _mavlink(mavlink)
	, _endpoint(endpoint)
	, _publishPeriodMs(publishPeriodMs)
	, _zmqContext(nullptr)
	, _zmqPub(nullptr)
	, _running(false)
{
}

SimulatorTelemetryPublisher::~SimulatorTelemetryPublisher()
{
	stop();
}

bool SimulatorTelemetryPublisher::start()
{
	if (_running) {
		return true;
	}

	_zmqContext = zmq_ctx_new();
	if (_zmqContext == nullptr) {
		logError() << "SimulatorTelemetryPublisher: zmq_ctx_new failed";
		return false;
	}

	_zmqPub = zmq_socket(_zmqContext, ZMQ_PUB);
	if (_zmqPub == nullptr) {
		logError() << "SimulatorTelemetryPublisher: zmq_socket failed";
		zmq_ctx_term(_zmqContext);
		_zmqContext = nullptr;
		return false;
	}

	int sndHwm = 8;
	zmq_setsockopt(_zmqPub, ZMQ_SNDHWM, &sndHwm, sizeof(sndHwm));

	if (zmq_bind(_zmqPub, _endpoint.c_str()) != 0) {
		logError() << "SimulatorTelemetryPublisher: zmq_bind failed for" << _endpoint << "error:" << zmq_strerror(zmq_errno());
		zmq_close(_zmqPub);
		_zmqPub = nullptr;
		zmq_ctx_term(_zmqContext);
		_zmqContext = nullptr;
		return false;
	}

	_running = true;
	_thread = std::make_unique<std::thread>(&SimulatorTelemetryPublisher::_run, this);

	logInfo() << "SimulatorTelemetryPublisher: publishing vehicle telemetry on" << _endpoint;
	return true;
}

void SimulatorTelemetryPublisher::stop()
{
	if (!_running) {
		return;
	}

	_running = false;

	if (_thread && _thread->joinable()) {
		_thread->join();
	}
	_thread.reset();

	if (_zmqPub != nullptr) {
		zmq_close(_zmqPub);
		_zmqPub = nullptr;
	}

	if (_zmqContext != nullptr) {
		zmq_ctx_term(_zmqContext);
		_zmqContext = nullptr;
	}
}

void SimulatorTelemetryPublisher::_run()
{
	while (_running) {
		auto position = _mavlink->telemetry().lastPosition();
		auto attitude = _mavlink->telemetry().lastAttitudeEuler();

		if (position.has_value() && attitude.has_value()) {
			char payload[320] = {};
			double timestampSec = secondsSinceEpoch();
			std::snprintf(
				payload,
				sizeof(payload),
				"vehicle_pose {\"t\":%.6f,\"lat_deg\":%.8f,\"lon_deg\":%.8f,\"alt_m\":%.3f,\"yaw_deg\":%.3f}",
				timestampSec,
				position->latitude,
				position->longitude,
				position->relativeAltitude,
				attitude->yawDegrees);

			const int rc = zmq_send(_zmqPub, payload, std::strlen(payload), ZMQ_DONTWAIT);
			if (rc < 0 && zmq_errno() != EAGAIN) {
				logWarn() << "SimulatorTelemetryPublisher: zmq_send failed:" << zmq_strerror(zmq_errno());
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(_publishPeriodMs));
	}
}
