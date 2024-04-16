#include "LogFileManager.h"
#include "formatString.h"
#include "log.h"

#include <chrono>
#include <iomanip>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

namespace bf = boost::filesystem;
namespace bs = boost::system;

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

    auto now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto now_utc    = *std::gmtime(&now_time_t);

    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M", &now_utc);

    _logDir = formatString("%s/%s%s", _homeDir.c_str(), _logDirPrefix, buffer);

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
