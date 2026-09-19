#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <list>

class LogFileManager
{
public:
	static LogFileManager* instance();

	/// (completed, total, message) as an operation advances; total is fixed for the call.
	using ProgressFn = std::function<void(uint32_t, uint32_t, const std::string&)>;

	typedef enum {
		DETECTORS,
		RAW_CAPTURE,
		ROTATION
	} LogType_t;

 void detectorsStarted();
	void detectorsStopped();
    bool detectorsLogging() const { return !_logDirDetectors.empty(); }

	void rawCaptureStarted();

	void rotationStarted();
	void rotationStopped();
	bool rotationActive() const { return !_logDirRotation.empty(); }

	enum class LogOpResult {
		Failed,
		Done,
		NothingToDo
	};

	/// Copies every log directory to the flash drive, one file per progress step,
	/// then unmounts it on the rPi.
	LogOpResult saveLogsToSDCard(const ProgressFn& progress = {});
	/// Deletes every log directory, one directory per progress step.
	LogOpResult cleanLocalLogs(const ProgressFn& progress = {});

	/// Check disk free space and prune oldest log directories if below threshold.
	/// @param minFreePercent  Trigger cleanup when free space falls below this (default 25%)
	/// @param targetFreePercent  Keep deleting until free space reaches this (default 30%)
	/// @param minKeepDirs  Always keep at least this many log directories (default 5)
	/// @return Number of directories removed
	unsigned int pruneOnDiskPressure(double minFreePercent = 25.0, double targetFreePercent = 30.0, unsigned int minKeepDirs = 5);

	std::string filename	(LogType_t logType, const char* root, const char* extension);
	std::string logDir		(LogType_t logType) const;
	/// Directory the controller log is mirrored into: detectors dir while detecting,
	/// else the rotation dir, else the most recently closed session dir so the
	/// tail of a session (FINISH ack, STOPPED, outcome replays, the next tag
	/// upload) is captured too. Empty only before the first session or after the
	/// logs are deleted.
	std::string controllerLogDir() const
	{
		if (!_logDirDetectors.empty()) return _logDirDetectors;
		if (!_logDirRotation.empty())  return _logDirRotation;
		return _logDirClosed;
	}

private:
	LogFileManager();

	void _createLogDir(LogType_t logType);
	std::string _getSDCardPath();
	/// complete (optional) is cleared if the listing could not be read in full.
	std::list<std::string> _listLogFileDirs(bool* complete = nullptr);

	std::string _homeDir;
	std::string _logsRoot;
	std::string _logDirDetectors;
	std::string _logDirRawCapture;
	std::string _logDirRotation;
	std::string _logDirClosed;      // last detectors/rotation dir after it ended

	static const std::string _logsDirPrefix;

	static LogFileManager* _instance;
};
