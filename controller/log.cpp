#include "log.h"
#include "LogFileManager.h"
#include "MavlinkSystem.h"
#include "formatString.h"

#include <cerrno>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_GRAY "\x1b[37m"
#define ANSI_COLOR_RESET "\x1b[0m"

std::mutex LogDetailed::_logMutex;

LogDetailed::LogDetailed(const char* filename, int filenumber)
    : _s                ()
    , _caller_filename  (filename)
    , _caller_filenumber(filenumber)
{

}

LogDetailed::~LogDetailed()
{
    _logMutex.lock();

    std::stringstream sStream;

    switch (_log_level) {
        case LogLevel::Debug:
            set_color(LogColor::Green, sStream);
            break;
        case LogLevel::Info:
            set_color(LogColor::Blue, sStream);
            break;
        case LogLevel::Warn:
            set_color(LogColor::Yellow, sStream);
            break;
        case LogLevel::Err:
            set_color(LogColor::Red, sStream);
            break;
    }


    // UTC, 24-hour: logs come from rPi flights and SITL desktops in different
    // timezones and are analyzed elsewhere; the detector and jsonl are UTC too.
    time_t rawtime;
    time(&rawtime);
    struct tm* timeinfo = gmtime(&rawtime);
    char time_buffer[10]{}; // We need 8 characters + \0
    strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", timeinfo);
    sStream << "[" << time_buffer;

    switch (_log_level) {
        case LogLevel::Debug:
            sStream << "|D] ";
            break;
        case LogLevel::Info:
            sStream << "|I] ";
            break;
        case LogLevel::Warn:
            sStream << "|W] ";
            break;
        case LogLevel::Err:
            sStream << "|E] ";
            break;
    }

    set_color(LogColor::Reset, sStream);

    sStream << " " << _s.str() << " (" << _caller_filename << ":" << std::dec << _caller_filenumber << ")";

    std::cout << sStream.str() << std::endl;

    // Latched so a dead log directory produces one report, not one per line.
    static std::string sFailedLogDir; // guarded by _logMutex
    std::string operatorError;

    const std::string controllerLogDir = LogFileManager::instance()->controllerLogDir();
    if (!controllerLogDir.empty()) {
        const std::string logPath = controllerLogDir + "/MavlinkTagController.log";
        // POSIX I/O rather than ofstream: errno is guaranteed here, and a short
        // write (ENOSPC) is detectable instead of being swallowed by operator<<.
        const char* failure = nullptr;
        const int fd = ::open(logPath.c_str(), O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0) {
            failure = strerror(errno);
        } else {
            const std::string line = sStream.str() + '\n';
            size_t written = 0;
            while (written < line.size()) {
                const ssize_t n = ::write(fd, line.data() + written, line.size() - written);
                if (n < 0) {
                    if (errno == EINTR) continue;
                    failure = strerror(errno);
                    break;
                }
                if (n == 0) {
                    // No progress and no error: only errno from a -1 is meaningful.
                    failure = "short write";
                    break;
                }
                written += static_cast<size_t>(n);
            }
            // Some filesystems report deferred write errors only from close().
            if (::close(fd) != 0 && failure == nullptr) {
                failure = strerror(errno);
            }
        }
        if (failure == nullptr) {
            sFailedLogDir.clear();
        } else if (sFailedLogDir != controllerLogDir) {
            sFailedLogDir = controllerLogDir;
            operatorError = formatString("Controller log not being saved: %s: %s",
                                         logPath.c_str(), failure);
        }
    }

    _logMutex.unlock();

    // sendStatusText logs, which re-enters this destructor: must run unlocked.
    if (!operatorError.empty()) {
        MavlinkSystem::instance()->sendStatusText(operatorError, MAV_SEVERITY_ERROR);
    }
}

void set_color(LogColor LogColor, std::stringstream& s)
{
    switch (LogColor) {
        case LogColor::Red:
            s << ANSI_COLOR_RED;
            break;
        case LogColor::Green:
            s << ANSI_COLOR_GREEN;
            break;
        case LogColor::Yellow:
            s << ANSI_COLOR_YELLOW;
            break;
        case LogColor::Blue:
            s << ANSI_COLOR_BLUE;
            break;
        case LogColor::Gray:
            s << ANSI_COLOR_GRAY;
            break;
        case LogColor::Reset:
            s << ANSI_COLOR_RESET;
            break;
    }
}
