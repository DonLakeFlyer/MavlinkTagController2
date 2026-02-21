#pragma once

#include <string>
#include <list>

class LogFileManager
{
public:
	static LogFileManager* instance();

	typedef enum {
		DETECTORS,
		RAW_CAPTURE
	} LogType_t;

	void detectorsStarted();
	void detectorsStopped();
    bool detectorsLogging() const { return !_logDirDetectors.empty(); }

	void rawCaptureStarted();

	void saveLogsToSDCard();
	void cleanLocalLogs();

	std::string filename	(LogType_t logType, const char* root, const char* extension);
	std::string logDir		(LogType_t logType) const;

private:
	LogFileManager();

	void _createLogDir(LogType_t logType);
	std::string _getSDCardPath();
	std::list<std::string> _listLogFileDirs();
	
	std::string _homeDir;
	std::string _logDirDetectors;
	std::string _logDirRawCapture;

	static const std::string _logsDirPrefix;

	static LogFileManager* _instance;
};
