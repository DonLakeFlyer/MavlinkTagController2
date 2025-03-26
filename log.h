#pragma once

#include <sstream>
#include <vector>
#include <iostream>
#include <ctime>
#include <mutex>
#include <functional>

// Remove path and extract only filename.
#define FILENAME \
    (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)

#define call_user_callback(...) call_user_callback_located(FILENAME, __LINE__, __VA_ARGS__)

#define logDebug()  LogDebugDetailed(FILENAME, __LINE__)
#define logInfo()   LogInfoDetailed (FILENAME, __LINE__)
#define logWarn()   LogWarnDetailed (FILENAME, __LINE__)
#define logError()  LogErrDetailed  (FILENAME, __LINE__)

enum class LogColor { Red, Green, Yellow, Blue, Gray, Reset };
enum class LogLevel : int { Debug = 0, Info = 1, Warn = 2, Err = 3 };

void set_color(LogColor LogColor, std::stringstream& s);

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
        for (auto value : vector) {
            _s << value << ", ";
        }   
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

class LogInfoDetailed : public LogDetailed {
public:
    LogInfoDetailed(const char* filename, int filenumber) 
        : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Info;
    }
};

class LogWarnDetailed : public LogDetailed {
public:
    LogWarnDetailed(const char* filename, int filenumber) 
        : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Warn;
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
