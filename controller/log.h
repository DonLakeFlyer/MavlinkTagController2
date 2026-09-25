#pragma once

#include <sstream>
#include <vector>
#include <iostream>
#include <ctime>
#include <mutex>
#include <functional>

#include "logLevel.h"

// Remove path and extract only filename.
#define FILENAME \
    (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)

#define call_user_callback(...) call_user_callback_located(FILENAME, __LINE__, __VA_ARGS__)

// Three levels. Debug and Error are always written; Verbose (per-frame,
// per-cycle chatter) only when enabled with --verbose.
#define logDebug()    LogDebugDetailed  (FILENAME, __LINE__)
#define logError()    LogErrDetailed    (FILENAME, __LINE__)
#define logVerbose()  LogVerboseDetailed(FILENAME, __LINE__)

enum class LogLevel : int { Verbose = 0, Debug = 1, Err = 2 };

class LogDetailed {
public:
    LogDetailed(const char* filename, int filenumber);
    LogDetailed(const LogDetailed&) = delete;

    virtual ~LogDetailed();

    void operator=(const LogDetailed&) = delete;

    LogDetailed& operator<<(uint8_t& x)
    {
        _s << (unsigned int)x << " ";
        return *this;
    }

    template<typename T> LogDetailed& operator<<(const T& x)
    {
        _s << x << " ";
        return *this;
    }

    template<typename T> LogDetailed& operator<<(const std::vector<T>& vector)
    {
        const char* sep = "";
        for (const auto& value : vector) {
            _s << sep << value;
            sep = ", ";
        }
        _s << " ";
        return *this;
    }

protected:
    LogLevel _log_level = LogLevel::Debug;

private:
    std::stringstream   _s;
    const char*         _caller_filename;
    int                 _caller_filenumber;

    static std::mutex   _logMutex;
};

class LogDebugDetailed : public LogDetailed {
public:
    LogDebugDetailed(const char* filename, int filenumber)
        : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Debug;
    }
};

class LogVerboseDetailed : public LogDetailed {
public:
    LogVerboseDetailed(const char* filename, int filenumber)
        : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Verbose;
    }
};

class LogErrDetailed : public LogDetailed {
public:
    LogErrDetailed(const char* filename, int filenumber)
        : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Err;
    }
};
