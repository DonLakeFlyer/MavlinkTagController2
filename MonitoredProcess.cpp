#include "MonitoredProcess.h"
#include "log.h"
#include "MavlinkSystem.h"
#include "formatString.h"

#include <string>
#include <iostream>
#include <thread>
#include <filesystem>

MonitoredProcess::MonitoredProcess(
		MavlinkSystem*					mavlink,
		const char* 					name, 
		const char* 					command, 
		const char* 					logPath, 
		IntermediatePipeType			intermediatePipeType,
		bp::pipe* 						intermediatePipe,
		bool							rawCaptureProcess)
	: _mavlink				(mavlink)
	, _name					(name)
	, _command				(command)
	, _logPath				(logPath)
	, _intermediatePipeType	(intermediatePipeType)
	, _intermediatePipe		(intermediatePipe)
	, _rawCaptureProcess	(rawCaptureProcess)
{

}

void MonitoredProcess::start(void)
{
	// FIXME: We leak these thread objects
    _thread = new std::thread(&MonitoredProcess::_run, this);
    _thread->detach();
}

void MonitoredProcess::stop(void)
{
	logDebug() << "MonitoredProcess::stop _name _childProcess:_childProcess.running" 
		<< _name
		<< _childProcess
		<< (_childProcess ? _childProcess->running() : false);

	if (_childProcess) {
		_stopped = true;
		try {
			_childProcess->terminate();
		} catch(bp::process_error& e) {
			logError() << "MonitoredProcess::run terminate threw process_error exception\n" 
				<< "\terror: " << e.what() << "\n"
				<< "\tcommand: " << _command;
		}
	}
}

void MonitoredProcess::_run(void)
{
	std::string statusStr("Process start: ");
	statusStr.append(_name);

	logInfo() << statusStr << "'" << _command.c_str() << "' >" << _logPath.c_str();
	_mavlink->sendStatusText(statusStr.c_str());

	if (_rawCaptureProcess) {
		statusStr = "#Capture started";
		_mavlink->sendStatusText(statusStr.c_str());
	}

	std::filesystem::remove(_logPath);

	try {
		switch (_intermediatePipeType ) {
			case NoPipe:
				_childProcess = new bp::child(_command.c_str(), bp::std_out > _logPath, bp::std_err > _logPath);
				break;
			case InputPipe:
				_childProcess = new bp::child(_command.c_str(), bp::std_in < *_intermediatePipe, bp::std_out > _logPath, bp::std_err > _logPath);
				break;
			case OutputPipe:
				_childProcess = new bp::child(_command.c_str(), bp::std_out > *_intermediatePipe, bp::std_err > _logPath);
				break;
		}
	} catch(bp::process_error& e) {
		logError() << "MonitoredProcess::run boost::process:child threw process_error exception\n" 
            << "\terror: " << e.what() << "\n"
            << "\tcommand: " << _command;
		auto statusStr = formatString("#Process start failed: %s", _name.c_str());
		_mavlink->sendStatusText(statusStr, MAV_SEVERITY_ERROR);
		delete this;
		return;
	}

	int result = 255;

	if (_childProcess) {
		try {
			_childProcess->wait();
		} catch(bp::process_error& e) {
			// This is ok since child process is gone
		}
		result = _childProcess->exit_code();
	}

	if (result == 0) {
		statusStr = "Process end: ";
	} else if (_stopped) {
		statusStr = "Process stopped: ";
	} else {
		char numStr[21];

		statusStr = "Process fail: ";
		snprintf(numStr, sizeof(numStr), "%d", result);
		statusStr.append(numStr);
		statusStr.append(" ");
	}
	statusStr.append(_name);
	logError() << statusStr;
	_mavlink->sendStatusText(statusStr, (result == 0 || _stopped) ? MAV_SEVERITY_INFO : MAV_SEVERITY_ERROR);

	if (_rawCaptureProcess) {
		_mavlink->setHeartbeatStatus(HEARTBEAT_STATUS_HAS_TAGS);
		_mavlink->sendStatusText("#Capture complete", MAV_SEVERITY_INFO);
	}

	delete _childProcess;
	_childProcess = NULL;

	_stopped = false;

    delete this;   
}