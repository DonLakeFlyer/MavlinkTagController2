#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

class MavlinkSystem;

class SimulatorTelemetryPublisher
{
public:
	SimulatorTelemetryPublisher(MavlinkSystem* mavlink, const std::string& endpoint, int publishPeriodMs);
	~SimulatorTelemetryPublisher();

	bool start();
	void stop();

private:
	void _run();

	MavlinkSystem* _mavlink;
	std::string _endpoint;
	int _publishPeriodMs;
	void* _zmqContext;
	void* _zmqPub;
	std::unique_ptr<std::thread> _thread;
	std::atomic_bool _running;
};
