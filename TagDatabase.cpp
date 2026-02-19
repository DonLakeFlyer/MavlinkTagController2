#include "TagDatabase.h"
#include "log.h"
#include "formatString.h"
#include "LogFileManager.h"

#include <algorithm>
#include <numeric>
#include <functional>
#include <errno.h>
#include <cstring>
#include <math.h>

std::string TagDatabase::detectorConfigFileName(const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel) const
{
    auto logFileManager = LogFileManager::instance();
    auto root = formatString("detector_%d", tagInfo.id + (secondaryChannel ? 1 : 0));
    return logFileManager->filename(LogFileManager::DETECTORS, root.c_str(), "config");
}

bool TagDatabase::_writeDetectorConfig(const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel, bool isHFMode) const
{
    auto logFileManager = LogFileManager::instance();

    std::string configPath  = detectorConfigFileName(tagInfo, secondaryChannel);

    FILE* fp = fopen(configPath.c_str(), "w");
    if (fp == NULL) {
        logError() << "_writeDetectorConfig fopen failed -" << strerror(errno);
        return false;
    }

    int     secondaryChannelIncrement   = secondaryChannel ? 1 : 0;
    auto    tagId                       = tagInfo.id + secondaryChannelIncrement;
    double  channelCenterFreqMHz        = double(tagInfo.channelizer_channel_center_frequency_hz) / 1000000.0;
    double  tagFreqMHz                  = double(tagInfo.frequency_hz) / 1000000.0;
    auto    tip_msecs                   = secondaryChannel ? tagInfo.intra_pulse2_msecs : tagInfo.intra_pulse1_msecs;
    auto    portData                    = isHFMode ? (10000 + secondaryChannelIncrement) : (20000 + ((tagInfo.channelizer_channel_number - 1) * 2) + secondaryChannelIncrement);
    auto    sampleRate                  = isHFMode ? 3840 : 3750;
    auto    tip                         = double(tip_msecs) / 1000.0;
    auto    logDir                      = logFileManager->logDir(LogFileManager::DETECTORS);

    fprintf(fp, "##################################################\n");
    fprintf(fp, "ID:\t%d\n",                                tagId);
    fprintf(fp, "channelCenterFreqMHz:\t%f\n",              channelCenterFreqMHz);
    fprintf(fp, "ipData:\t127.0.0.1\n");
    fprintf(fp, "portData:\t%d\n",                          portData);
    fprintf(fp, "Fs:\t%d\n",                                sampleRate);
    fprintf(fp, "tagFreqMHz:\t%f\n",                            tagFreqMHz);
    fprintf(fp, "tp:\t%f\n",                                    tagInfo.pulse_width_msecs / 1000.0);
    fprintf(fp, "tip:\t%f\n",                                   tip);
    fprintf(fp, "tipu:\t%f\n",                                  tagInfo.intra_pulse_uncertainty_msecs / 1000.0);
    fprintf(fp, "tipj:\t%f\n",                                  tagInfo.intra_pulse_jitter_msecs / 1000.0);
    fprintf(fp, "K:\t%u\n",                                     tagInfo.k);
    fprintf(fp, "opMode:\tfreqSearchHardLock\n");
    fprintf(fp, "excldFreqs:\t[Inf, -Inf]\n");
    fprintf(fp, "falseAlarmProb:\t%f\n",                        tagInfo.false_alarm_probability);
    fprintf(fp, "dataRecordPath:\t%s/data_record_%d.bin\n",     logDir.c_str(), tagId);
    fprintf(fp, "logPath:\t%s\n",                               logDir.c_str());
    fprintf(fp, "startIndex:\t%d\n",                            1);
    fprintf(fp, "ipCntrl:\t127.0.0.1\n");
    fprintf(fp, "portCntrl:\t30000\n");
    fprintf(fp, "processedOuputPath:\t%s\n",                    logDir.c_str());
    fprintf(fp, "ros2enable:\tfalse\n");
    fprintf(fp, "startInRunState:\ttrue\n");
    fprintf(fp, "timeStamp:\t1646403180.469\n");

    fclose(fp);

    // Log the entire file contents for debugging ease

    fp = fopen(configPath.c_str(), "r");
    if (fp == NULL) {
        logError() << "_writeDetectorConfig fopen failed for log output -" << strerror(errno);
        return false;
    }

    std::string fileContents;
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), fp) != NULL) {
        fileContents += buffer;
    }

    logInfo() << "DETECTOR CONFIG:" << configPath;
    logInfo() << fileContents;

    fclose(fp);

    return true;
}

bool TagDatabase::writeDetectorConfigs(bool isHFMode) const
{
    for (const auto& tagInfo : *this) {
        _writeDetectorConfig(tagInfo, false, isHFMode);
        if (tagInfo.intra_pulse2_msecs != 0) {
            _writeDetectorConfig(tagInfo, true, isHFMode);
        }
    }

    return true;
}

std::string TagDatabase::channelizerCommandLine() const
{
    std::string commandLine;

    for (const TunnelProtocol::TagInfo_t& tagInfo : *this) {
        commandLine += formatString("%d ", tagInfo.channelizer_channel_number * (tagInfo.intra_pulse2_msecs != 0 ? -1 : 1));
    }

    return commandLine;
}
