#include "LogFileManager.h"
#include "formatString.h"
#include "log.h"
#include "MavlinkSystem.h"

#include <chrono>
#include <iomanip>
#include <ctime>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

namespace bf = boost::filesystem;
namespace bs = boost::system;

extern MavlinkSystem* globalMavlinkSystem;

LogFileManager* LogFileManager::_instance       = nullptr;
const char* 	LogFileManager::_logDirPrefix   = "Logs-";

LogFileManager* LogFileManager::instance()
{
    if (_instance == nullptr) {
        _instance = new LogFileManager();
    }
    return _instance;
}

LogFileManager::LogFileManager()
{
    _homeDir = std::string(getenv("HOME"));
}

void LogFileManager::detectorsStarted()
{
    _detectorsRunning = true;

    std::time_t vehicleTime;
    if (globalMavlinkSystem->vehicleEpochTime().has_value()) {
        vehicleTime = globalMavlinkSystem->vehicleEpochTime().value();
    } else {
        logWarn() << "Vehicle time not available, using rPi time";
        vehicleTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    }
    auto vehicleTimeUTC = *std::gmtime(&vehicleTime);

    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &vehicleTimeUTC);

    _logDir = formatString("%s/%s%s", _homeDir.c_str(), _logDirPrefix, buffer);
    logDebug() << "Creating new log directory:" << _logDir;

    bs::error_code errorCode;
    bf::create_directory(_logDir.c_str(), errorCode);
    if (errorCode) {
        logDebug() << "Failed to create directory " << _logDir << ": " << errorCode.message();
    }
}

void LogFileManager::detectorsStopped()
{
    _detectorsRunning = false;
}

std::string LogFileManager::filename(const char* root, const char* extension)
{
    return formatString("%s/%s.%s", _logDir.c_str(), root, extension);
}
