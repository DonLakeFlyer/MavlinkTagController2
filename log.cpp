#include "log.h"
#include "LogFileManager.h"
#include "formatString.h"

#include <fstream>
#include <cstdio>

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


    // Time output taken from:
    // https://stackoverflow.com/questions/16357999#answer-16358264
    time_t rawtime;
    time(&rawtime);
    struct tm* timeinfo = localtime(&rawtime);
    char time_buffer[10]{}; // We need 8 characters + \0
    strftime(time_buffer, sizeof(time_buffer), "%I:%M:%S", timeinfo);
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

    auto logFileManager = LogFileManager::instance();
    if (logFileManager->detectorsLogging()) {
        std::ofstream logFile(logFileManager->filename(LogFileManager::DETECTORS, "MavlinkTagController", "log"), std::ios_base::app);
        logFile << sStream.str() << std::endl;
    }

    _logMutex.unlock();
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
