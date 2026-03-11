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

    fs::directory_iterator end_itr;
    for (fs::directory_iterator itr(_homeDir); itr != end_itr; ++itr) {
        if (fs::is_directory(itr->status())) {
            std::string dirName = itr->path().filename().string();
            if (dirName.find(_logsDirPrefix) == 0) {
                logDirs.push_back(dirName);
            }
        }
    }

    logDebug() << "Found " << logDirs.size() << " log directories in " << _homeDir;

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
    logInfo() << "Saving logs to SD card";

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

    MavlinkSystem::instance()->sendStatusText("#Saving logs", MAV_SEVERITY_INFO);

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

    auto unmountCommand = formatString("sudo umount %s", sdCardPath.c_str());
    int unmountResult = std::system(unmountCommand.c_str());
    if (unmountResult == 0) {
        MavlinkSystem::instance()->sendStatusText("#Log save complete", MAV_SEVERITY_INFO);
    } else {
        MavlinkSystem::instance()->sendStatusText("#Unmount failed", MAV_SEVERITY_ERROR);
    }
}

void LogFileManager::cleanLocalLogs()
{
    logInfo() << "Cleaning local logs";

    auto logDirs = _listLogFileDirs();
    if (logDirs.empty()) {
        logInfo() << "No log directories found";
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

    logInfo() << "Local logs deleted";
}

unsigned int LogFileManager::pruneOnDiskPressure(double minFreePercent, double targetFreePercent, unsigned int minKeepDirs)
{
    // Check free space on the filesystem containing the home directory
    std::error_code ec;
    auto spaceInfo = fs::space(_homeDir, ec);
    if (ec || spaceInfo.capacity == 0) {
        logWarn() << "LogRetention: unable to query disk space: " << ec.message();
        return 0;
    }

    double freePercent = 100.0 * static_cast<double>(spaceInfo.available) / static_cast<double>(spaceInfo.capacity);
    double totalGB     = static_cast<double>(spaceInfo.capacity) / (1024.0 * 1024.0 * 1024.0);
    double freeGB      = static_cast<double>(spaceInfo.available) / (1024.0 * 1024.0 * 1024.0);

    logInfo() << "LogRetention: disk " << std::fixed << std::setprecision(1)
              << freeGB << " GB free / " << totalGB << " GB total (" << freePercent << "%)";

    if (freePercent >= minFreePercent) {
        logInfo() << "LogRetention: free space " << std::fixed << std::setprecision(1)
                  << freePercent << "% >= " << minFreePercent << "% threshold, no cleanup needed";
        return 0;
    }

    logWarn() << "LogRetention: free space " << std::fixed << std::setprecision(1)
              << freePercent << "% < " << minFreePercent << "% threshold, starting cleanup";

    // Get log directories sorted oldest-first by timestamp suffix.
    // Names are e.g. "Logs-Detectors-2026-03-10_14-30-22" and
    // "Logs-RawCapture-2026-03-09_08-00-00".  A plain lexicographic sort
    // would interleave by prefix ("Detectors" < "RawCapture"), so we sort
    // by the trailing YYYY-MM-DD_HH-MM-SS timestamp instead.
    auto logDirs = _listLogFileDirs();
    logDirs.sort([](const std::string& a, const std::string& b) {
        // Timestamp suffix is always the last 19 characters (YYYY-MM-DD_HH-MM-SS)
        const size_t tsLen = 19;  // strlen("2026-03-10_14-30-22")
        std::string tsA = a.size() >= tsLen ? a.substr(a.size() - tsLen) : a;
        std::string tsB = b.size() >= tsLen ? b.substr(b.size() - tsLen) : b;
        return tsA < tsB;
    });

    unsigned int removed = 0;

    while (!logDirs.empty() && logDirs.size() > minKeepDirs) {
        // Re-check free space after each deletion
        spaceInfo = fs::space(_homeDir, ec);
        if (ec || spaceInfo.capacity == 0) {
            break;
        }
        freePercent = 100.0 * static_cast<double>(spaceInfo.available) / static_cast<double>(spaceInfo.capacity);

        if (freePercent >= targetFreePercent) {
            logInfo() << "LogRetention: free space recovered to " << std::fixed << std::setprecision(1)
                      << freePercent << "%, stopping cleanup";
            break;
        }

        // Skip if this is the currently active detector or raw-capture directory
        fs::path dirPath = _homeDir + "/" + logDirs.front();
        if (dirPath.string() == _logDirDetectors || dirPath.string() == _logDirRawCapture) {
            logDirs.pop_front();
            continue;
        }

        logInfo() << "LogRetention: removing " << logDirs.front()
                  << " (free space " << std::fixed << std::setprecision(1)
                  << freePercent << "% < target " << targetFreePercent << "%)";

        std::error_code removeEc;
        fs::remove_all(dirPath, removeEc);
        if (removeEc) {
            logError() << "LogRetention: failed to remove " << dirPath << ": " << removeEc.message();
            break;  // Don't loop on a broken directory
        }

        logDirs.pop_front();
        removed++;
    }

    if (removed > 0) {
        spaceInfo = fs::space(_homeDir, ec);
        freePercent = (ec || spaceInfo.capacity == 0) ? 0.0
                      : 100.0 * static_cast<double>(spaceInfo.available) / static_cast<double>(spaceInfo.capacity);
        logInfo() << "LogRetention: pruned " << removed << " old log directories, free space now "
                  << std::fixed << std::setprecision(1) << freePercent << "%";
    } else if (freePercent < minFreePercent) {
        logWarn() << "LogRetention: disk still under pressure (" << std::fixed << std::setprecision(1)
                  << freePercent << "%) but only " << logDirs.size()
                  << " log dirs remain (min keep: " << minKeepDirs << ")";
    }

    return removed;
}
