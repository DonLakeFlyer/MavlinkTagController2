#include "LogFileManager.h"
#include "formatString.h"
#include "log.h"
#include "MavlinkSystem.h"
#include "platformHelpers.h"

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <vector>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace fs = std::filesystem;

namespace {

// No shell: the mount point is the flash drive's volume label, which may hold
// spaces or metacharacters. Returns the exit status, 128+signal, or -errno.
int runNoShell(std::vector<const char*> argv)
{
    std::vector<char*> args;
    for (const char* a : argv) {
        args.push_back(const_cast<char*>(a));
    }
    args.push_back(nullptr);

    pid_t pid = 0;
    const int spawnErr = posix_spawnp(&pid, args[0], nullptr, nullptr, args.data(), environ);
    if (spawnErr != 0) {
        return -spawnErr;
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        return -errno;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1;
}

} // namespace

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
    _homeDir = homeDir();
    _logsRoot = _homeDir + "/Logs";
    std::error_code errorCode;
    fs::create_directories(_logsRoot, errorCode);
    if (errorCode) {
        // Log::~Log() calls LogFileManager::instance(); _instance is not yet set
        // here, so logging would recurse into this constructor.
        std::fprintf(stderr, "Failed to create logs root %s: %s\n",
                     _logsRoot.c_str(), errorCode.message().c_str());
    }
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
    case ROTATION:
        if (!_logDirRotation.empty()) {
            logDebug() << "Previous rotation log directory not cleaned up, replacing";
            _logDirRotation.clear();
        }
        logDirPrefix = formatString("%s%s", _logsDirPrefix.c_str(), "Rotation");
        break;
    }

    auto vehicleTime = MavlinkSystem::instance()->vehicleTimeNow();
    struct tm vehicleTimeUTC;
    gmtime_r(&vehicleTime, &vehicleTimeUTC);

    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &vehicleTimeUTC);

    std::string logDir = formatString("%s/%s-%s", _logsRoot.c_str(), logDirPrefix.c_str(), buffer);

    std::error_code errorCode;
    fs::create_directories(logDir.c_str(), errorCode);
    if (errorCode) {
        logError() << "Failed to create directory " << logDir << ": " << errorCode.message();
    }

    switch (logType) {
    case DETECTORS:
        logDebug() << "Created new detectors log directory:" << logDir;
        _logDirDetectors = logDir;
        _logDirClosed.clear();
        break;
    case RAW_CAPTURE:
        logDebug() << "Created new raw capture log directory:" << logDir;
        _logDirRawCapture = logDir;
        break;
    case ROTATION:
        logDebug() << "Created new rotation log directory:" << logDir;
        _logDirRotation = logDir;
        _logDirClosed.clear();
        break;
    }

}

void LogFileManager::detectorsStarted()
{
    if (!_logDirRotation.empty()) {
        // Rotation: detectors persist across headings, so their logs live at the
        // rotation root; each detector creates heading-XXX/ for per-slice output.
        _logDirDetectors = _logDirRotation;
    } else {
        _createLogDir(DETECTORS);
    }
}

void LogFileManager::detectorsStopped()
{
    // Inside a rotation the detectors dir is the rotation dir, which stays open.
    if (_logDirRotation.empty() && !_logDirDetectors.empty()) {
        _logDirClosed = _logDirDetectors;
    }
    _logDirDetectors.clear();
}

void LogFileManager::rawCaptureStarted()
{
    _createLogDir(RAW_CAPTURE);
}

void LogFileManager::rotationStarted()
{
    _createLogDir(ROTATION);
}

void LogFileManager::rotationStopped()
{
    if (!_logDirRotation.empty()) {
        _logDirClosed = _logDirRotation;
    }
    _logDirRotation.clear();
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
    case ROTATION:
        logDir = _logDirRotation;
        break;
    }

    return formatString("%s/%s.%s", logDir.c_str(), root, extension);
}

std::string LogFileManager::logDir(LogFileManager::LogType_t logType) const
{
    if (logType == DETECTORS) {
        return _logDirDetectors;
    } else if (logType == ROTATION) {
        return _logDirRotation;
    } else {
        return _logDirRawCapture;
    }
}

std::list<std::string> LogFileManager::_listLogFileDirs(bool* complete)
{
    std::list<std::string> logDirs;
    if (complete) {
        *complete = true;
    }

    std::error_code ec;
    fs::directory_iterator end_itr;
    for (fs::directory_iterator itr(_logsRoot, ec); !ec && itr != end_itr; itr.increment(ec)) {
        std::error_code statusEc;
        const auto status = itr->status(statusEc);
        if (statusEc) {
            logDebug() << "Skipping " << itr->path() << ": " << statusEc.message();
            if (complete) {
                *complete = false;
            }
            continue;
        }
        if (fs::is_directory(status)) {
            std::string dirName = itr->path().filename().string();
            if (dirName.find(_logsDirPrefix) == 0) {
                logDirs.push_back(dirName);
            }
        }
    }
    if (ec) {
        logDebug() << "Error listing " << _logsRoot << ": " << ec.message();
        if (complete) {
            *complete = false;
        }
    }

    logDebug() << "Found " << logDirs.size() << " log directories in " << _logsRoot;

    return logDirs;
}

std::string LogFileManager::_getSDCardPath()
{
    if (!isRunningOnRPi()) {
        fs::path fakeSDCardDir = fs::path(_homeDir) / "fake-sdcard";
        std::error_code ec;
        fs::create_directories(fakeSDCardDir, ec);
        if (ec) {
            std::string errorMsg = "Unable to create fake SD card directory: " + fakeSDCardDir.string() + ": " + ec.message();
            logError() << errorMsg;
            MavlinkSystem::instance()->sendStatusText(errorMsg, MAV_SEVERITY_ALERT);
            return std::string();
        }
        logDebug() << "Not running on rPi, saving logs to" << fakeSDCardDir.string();
        return fakeSDCardDir.string();
    }

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

LogFileManager::LogOpResult LogFileManager::saveLogsToSDCard(const ProgressFn& progress)
{
    logDebug() << "Saving logs to SD card";

    std::string sdCardPath = _getSDCardPath();
    if (sdCardPath.empty()) {
        return LogOpResult::Failed;
    }

    bool listingComplete = true;
    auto logDirs = _listLogFileDirs(&listingComplete);
    if (!listingComplete) {
        // An unreadable root must not become "no logs to save".
        MavlinkSystem::instance()->sendStatusText("#Error during log save", MAV_SEVERITY_ERROR);
        return LogOpResult::Failed;
    }
    if (logDirs.empty()) {
        logDebug() << "No log directories found";
        return LogOpResult::NothingToDo;
    }

    // Count first so the GCS can show a fraction; the unmount is the last step.
    std::vector<fs::path> files;
    bool allOk = true;
    for (const auto& logDir: logDirs) {
        std::error_code ec;
        for (fs::recursive_directory_iterator it(_logsRoot + "/" + logDir, ec), end; !ec && it != end; it.increment(ec)) {
            std::error_code statEc;
            if (it->is_regular_file(statEc)) {
                files.push_back(it->path());
            } else if (statEc) {
                logError() << "Cannot stat " << it->path() << ": " << statEc.message();
                allOk = false;
            }
        }
        // A truncated walk must not be reported as a complete save.
        if (ec) {
            logError() << "Failed to walk " << logDir << ": " << ec.message();
            allOk = false;
        }
    }
    const bool     onRPi   = isRunningOnRPi();
    const uint32_t total   = static_cast<uint32_t>(files.size()) + (onRPi ? 1 : 0);
    uint32_t       done    = 0;

    for (const auto& src: files) {
        // Pure string op: src came from an iterator rooted at _logsRoot, and fs::relative can throw.
        const fs::path relative = src.lexically_relative(_logsRoot);
        const fs::path dst      = fs::path(sdCardPath) / relative;
        std::error_code ec;
        fs::create_directories(dst.parent_path(), ec);
        if (!ec) {
            fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
        }
        if (ec) {
            logError() << "Failed to copy " << src << " to " << dst << ": " << ec.message();
            allOk = false;
        }
        if (progress) {
            progress(++done, total, relative.string());
        }
    }
    if (!allOk) {
        MavlinkSystem::instance()->sendStatusText("#Error during log save", MAV_SEVERITY_ERROR);
    }

    if (!onRPi) {
        if (allOk) {
            MavlinkSystem::instance()->sendStatusText("#Log save complete", MAV_SEVERITY_INFO);
        }
        return allOk ? LogOpResult::Done : LogOpResult::Failed;
    }

    if (progress) {
        progress(done, total, "Unmounting flash drive");
    }
    // -n: fail instead of prompting when the passwordless sudoers rule is missing.
    const int unmountResult = runNoShell({"sudo", "-n", "umount", sdCardPath.c_str()});
    if (unmountResult == 0) {
        if (progress) {
            progress(++done, total, "Flash drive unmounted");
        }
        if (allOk) {
            MavlinkSystem::instance()->sendStatusText("#Log save complete", MAV_SEVERITY_INFO);
        }
        return allOk ? LogOpResult::Done : LogOpResult::Failed;
    }
    if (unmountResult < 0) {
        logError() << "umount " << sdCardPath << " could not be run: " << strerror(-unmountResult);
    } else {
        logError() << "umount " << sdCardPath << " failed with status " << unmountResult;
    }
    MavlinkSystem::instance()->sendStatusText("#Unmount failed", MAV_SEVERITY_ERROR);
    return LogOpResult::Failed;
}

LogFileManager::LogOpResult LogFileManager::cleanLocalLogs(const ProgressFn& progress)
{
    logDebug() << "Cleaning local logs";
    // The closed session dir is about to be deleted; stop mirroring into it.
    _logDirClosed.clear();

    bool listingComplete = true;
    auto logDirs = _listLogFileDirs(&listingComplete);
    if (!listingComplete) {
        MavlinkSystem::instance()->sendStatusText("#Error during log delete", MAV_SEVERITY_ERROR);
        return LogOpResult::Failed;
    }
    if (logDirs.empty()) {
        logDebug() << "No log directories found";
        return LogOpResult::NothingToDo;
    }

    const uint32_t total = static_cast<uint32_t>(logDirs.size());
    uint32_t       done  = 0;
    bool           allOk = true;
    for (const auto& logDir: logDirs) {
        fs::path dirPath = _logsRoot + "/" + logDir;
        std::error_code errorCode;
        logDebug() << "Removing directory " << dirPath;
        fs::remove_all(dirPath, errorCode);
        if (errorCode) {
            logError() << "Failed to remove directory " << dirPath << ": " << errorCode.message();
            MavlinkSystem::instance()->sendStatusText("#Error during log delete", MAV_SEVERITY_ERROR);
            allOk = false;
        }
        if (progress) {
            progress(++done, total, logDir);
        }
    }

    if (allOk) {
        logDebug() << "Local logs deleted";
    } else {
        logDebug() << "Local log deletion incomplete";
    }
    return allOk ? LogOpResult::Done : LogOpResult::Failed;
}

unsigned int LogFileManager::pruneOnDiskPressure(double minFreePercent, double targetFreePercent, unsigned int minKeepDirs)
{
    // Check free space on the filesystem containing the home directory
    std::error_code ec;
    auto spaceInfo = fs::space(_homeDir, ec);
    if (ec || spaceInfo.capacity == 0) {
        logDebug() << "LogRetention: unable to query disk space: " << ec.message();
        return 0;
    }

    double freePercent = 100.0 * static_cast<double>(spaceInfo.available) / static_cast<double>(spaceInfo.capacity);
    double totalGB     = static_cast<double>(spaceInfo.capacity) / (1024.0 * 1024.0 * 1024.0);
    double freeGB      = static_cast<double>(spaceInfo.available) / (1024.0 * 1024.0 * 1024.0);

    logDebug() << "LogRetention: disk " << std::fixed << std::setprecision(1)
              << freeGB << " GB free / " << totalGB << " GB total (" << freePercent << "%)";

    if (freePercent >= minFreePercent) {
        logDebug() << "LogRetention: free space " << std::fixed << std::setprecision(1)
                  << freePercent << "% >= " << minFreePercent << "% threshold, no cleanup needed";
        return 0;
    }

    logDebug() << "LogRetention: free space " << std::fixed << std::setprecision(1)
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
            logDebug() << "LogRetention: free space recovered to " << std::fixed << std::setprecision(1)
                      << freePercent << "%, stopping cleanup";
            break;
        }

        // Skip if this is the currently active detector or raw-capture directory
        fs::path dirPath = _logsRoot + "/" + logDirs.front();
        if (dirPath.string() == _logDirDetectors || dirPath.string() == _logDirRawCapture) {
            logDirs.pop_front();
            continue;
        }
        if (dirPath.string() == _logDirClosed) {
            _logDirClosed.clear();
        }

        logDebug() << "LogRetention: removing " << logDirs.front()
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
        logDebug() << "LogRetention: pruned " << removed << " old log directories, free space now "
                  << std::fixed << std::setprecision(1) << freePercent << "%";
    } else if (freePercent < minFreePercent) {
        logDebug() << "LogRetention: disk still under pressure (" << std::fixed << std::setprecision(1)
                  << freePercent << "%) but only " << logDirs.size()
                  << " log dirs remain (min keep: " << minKeepDirs << ")";
    }

    return removed;
}
