#include "LogFileManager.h"
#include "formatString.h"
#include "log.h"
#include "MavlinkSystem.h"

#include <chrono>
#include <iomanip>
#include <ctime>
#include <filesystem>

#include <unistd.h>

namespace fs = std::filesystem;

LogFileManager* LogFileManager::_instance = nullptr;

const std::string LogFileManager::_logsDirPrefix = "Logs-";

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

void LogFileManager::_createLogDir(LogFileManager::LogType_t logType)
{
    std::string logDirPrefix;

    switch (logType) {
    case DETECTORS:
        if (!_logDirDetectors.empty()) {
            logError() << "Detectors already started";
            _logDirDetectors.clear();
        }
        logDirPrefix = formatString("%s%s", _logsDirPrefix.c_str(), "Detectors");
        break;
    case RAW_CAPTURE:
        if (!_logDirRawCapture.empty()) {
            // Raw capture log directory is created once per controller run
            return;
        }
        logDirPrefix = formatString("%s%s", _logsDirPrefix.c_str(), "RawCapture");
        break;
    }

    // rPi time should be synchronized with vehicle time
    auto vehicleTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto vehicleTimeUTC = *std::gmtime(&vehicleTime);

    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &vehicleTimeUTC);

    std::string logDir = formatString("%s/%s-%s", _homeDir.c_str(), logDirPrefix.c_str(), buffer);

    std::error_code errorCode;
    fs::create_directory(logDir.c_str(), errorCode);
    if (errorCode) {
        logDebug() << "Failed to create directory " << logDir << ": " << errorCode.message();
    }

    switch (logType) {
    case DETECTORS:
        logDebug() << "Created new detectors log directory:" << logDir;
        _logDirDetectors = logDir;
        break;
    case RAW_CAPTURE:
        logDebug() << "Created new raw capture log directory:" << logDir;
        _logDirRawCapture = logDir;
        break;
    }

}

void LogFileManager::detectorsStarted()
{
    _createLogDir(DETECTORS);
}

void LogFileManager::detectorsStopped()
{
    _logDirDetectors.clear();
}

void LogFileManager::rawCaptureStarted()
{
    _createLogDir(RAW_CAPTURE);
}

std::string LogFileManager::filename(LogType_t logType, const char* root, const char* extension)
{
    std::string logDir;

    switch (logType) {
    case DETECTORS:
        logDir = _logDirDetectors;
        break;
    case RAW_CAPTURE:
        logDir = _logDirRawCapture;
        break;
    }

    return formatString("%s/%s.%s", logDir.c_str(), root, extension);
}

std::string LogFileManager::logDir(LogFileManager::LogType_t logType) const
{
    if (logType == DETECTORS) {
        return _logDirDetectors;
    } else {
        return _logDirRawCapture;
    }
}

std::list<std::string> LogFileManager::_listLogFileDirs()
{
    std::list<std::string> logDirs;

    logDebug() << "Listing log directories in " << _homeDir;

    fs::directory_iterator end_itr;
    for (fs::directory_iterator itr(_homeDir); itr != end_itr; ++itr) {
        if (fs::is_directory(itr->status())) {
            logDebug() << "Found directory: " << itr->path().filename().string();
            std::string dirName = itr->path().filename().string();
            if (dirName.find(_logsDirPrefix) == 0) {
                logDirs.push_back(dirName);
            }
        }
    }

    return logDirs;
}

std::string LogFileManager::_getSDCardPath()
{
    fs::path rpiMediaDir("/media/pi");
    if (!fs::exists(rpiMediaDir)) {
        std::string errorMsg = "Unable to locate media directory: " + rpiMediaDir.string();
        logError() << errorMsg;
        MavlinkSystem::instance()->sendStatusText(errorMsg, MAV_SEVERITY_ALERT);
        return std::string();
    }

    uint dirCount = 0;
    std::string sdCardPath;
    std::string errorMsg;
    fs::directory_iterator end_itr;
    for (fs::directory_iterator itr(rpiMediaDir); itr != end_itr; ++itr) {
        if (fs::is_directory(itr->status())) {
            if (dirCount++ == 0) {
                sdCardPath = itr->path().string();
            }
        }
    }
    if (dirCount == 0) {
        errorMsg = "Flash drive not found";
    } else if (dirCount > 1) {
        errorMsg = "Multiple directories found in " + rpiMediaDir.string();
    }
    if (!errorMsg.empty()) {
        logError() << errorMsg;
        MavlinkSystem::instance()->sendStatusText(errorMsg, MAV_SEVERITY_ALERT);
        sdCardPath.clear();
    }

    return sdCardPath;
}

void LogFileManager::saveLogsToSDCard()
{
    std::string sdCardPath = _getSDCardPath();
    if (sdCardPath.empty()) {
        return;
    }

    auto logDirs = _listLogFileDirs();
    if (logDirs.empty()) {
        logInfo() << "No log directories found";
        MavlinkSystem::instance()->sendStatusText("#No logs to save", MAV_SEVERITY_INFO);
        return;
    }

    // Copy all directories in logDirs to //media/pi/LOGS
    for (const auto& logDir: logDirs) {
        fs::path srcDir = _homeDir + "/" + logDir;
        fs::path dstDir = sdCardPath + "/" + logDir;
        std::error_code errorCode;
        logInfo() << "Copying directory " << srcDir << " to " << dstDir;
        fs::copy(srcDir, dstDir, fs::copy_options::overwrite_existing | fs::copy_options::recursive, errorCode);
        if (errorCode) {
            logError() << "Failed to copy directory " << srcDir << " to " << dstDir << ": " << errorCode.message();
            MavlinkSystem::instance()->sendStatusText("#Error during log save", MAV_SEVERITY_ERROR);
        }
    }

    sync(); // Flush all disks

    MavlinkSystem::instance()->sendStatusText("#Logs saved to flash drive", MAV_SEVERITY_INFO);
}

void LogFileManager::cleanLocalLogs()
{
    auto logDirs = _listLogFileDirs();
    if (logDirs.empty()) {
        logInfo() << "No log directories found";
        MavlinkSystem::instance()->sendStatusText("#No logs to delete", MAV_SEVERITY_INFO);
        return;
    }

    // Delete all directories in logDirs
    for (const auto& logDir: logDirs) {
        fs::path dirPath = _homeDir + "/" + logDir;
        std::error_code errorCode;
        logInfo() << "Removing directory " << dirPath;
        if (!fs::remove_all(dirPath, errorCode)) {
            logError() << "Failed to remove directory " << dirPath << ": " << errorCode.message();
            MavlinkSystem::instance()->sendStatusText("#Error during log delete", MAV_SEVERITY_ERROR);
        }
    }

    MavlinkSystem::instance()->sendStatusText("#Logs deleted", MAV_SEVERITY_INFO);
}