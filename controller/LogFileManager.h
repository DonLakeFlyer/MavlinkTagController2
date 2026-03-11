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

	/// Check disk free space and prune oldest log directories if below threshold.
	/// @param minFreePercent  Trigger cleanup when free space falls below this (default 25%)
	/// @param targetFreePercent  Keep deleting until free space reaches this (default 30%)
	/// @param minKeepDirs  Always keep at least this many log directories (default 5)
	/// @return Number of directories removed
	unsigned int pruneOnDiskPressure(double minFreePercent = 25.0, double targetFreePercent = 30.0, unsigned int minKeepDirs = 5);

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
