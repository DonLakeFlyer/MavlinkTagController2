#pragma once

#include <vector>
#include <cstdint>
#include <string>

#include "TunnelProtocol.h"

class TagDatabase : public std::vector<TunnelProtocol::TagInfo_t>
{
public:
    TagDatabase() = default;

    std::string detectorConfigFileName  (const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel) const;
    bool        writeDetectorConfigs    () const;
    std::string channelizerCommandLine  () const;

private:
    bool        _writeDetectorConfig    (const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel) const;
};
