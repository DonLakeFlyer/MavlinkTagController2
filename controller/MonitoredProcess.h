#pragma once

#include <string>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>

#include "boost_process_compat.h"

class MavlinkSystem;

class MonitoredProcess : public std::enable_shared_from_this<MonitoredProcess>
{
public:
	enum IntermediatePipeType {
		NoPipe,
		InputPipe,
		OutputPipe,
	};

    MonitoredProcess(
        MavlinkSystem* mavlink,
        const char* name,
        const char* command,
        const char* logPath,
        IntermediatePipeType intermediatePipeType,
        bp::pipe* intermediatePipe,
        bool rawCaptureProcess = false,
        std::function<void(int)> failureCallback = {});

	void start 	(void);
	/// terminate() then waitForExit(); the exit is logged before this returns.
	void stop	(std::chrono::milliseconds timeout = std::chrono::seconds(5));
	/// Asks the child to exit without waiting. Pair with waitForExit().
	void terminate(void);
	/// Blocks until _run() has logged the exit, or timeout (then warns).
	void waitForExit(std::chrono::milliseconds timeout = std::chrono::seconds(5));

private:
	void _run(void);
	void _signalExited(void);

	MavlinkSystem*					_mavlink;
	std::string						_name;
	std::string 					_command;
	std::string						_logPath;
	bp::child*						_childProcess 	= NULL;
	bool							_stopped		= false;
	std::mutex						_exitMutex;
	std::condition_variable			_exitCondition;
	bool							_exited			= false;
	IntermediatePipeType			_intermediatePipeType;
	bp::pipe*						_intermediatePipe;
	bool							_rawCaptureProcess;
    std::function<void(int)> _failureCallback;
};
