#pragma once

#include <string>

class LogFileManager
{
public:
	static LogFileManager* instance();

	void 		detectorsStarted();
	void 		detectorsStopped();
    bool        detectorsRunning() const { return _detectorsRunning; }

	std::string filename	(const char* root, const char* extension);
	std::string logDir		() const { return _logDir; }

	static std::string logDirPrefix() { return _logDirPrefix; }

private:
	LogFileManager();
	
	std::string _homeDir;
	std::string _logDir;
    bool        _detectorsRunning = false;

	static LogFileManager* 	_instance;
	static const char* 		_logDirPrefix;
};
