#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "TunnelProtocol.h"

class TagDatabase : public std::vector<TunnelProtocol::TagInfo_t>
{
public:
    TagDatabase() = default;

    // Result of ingesting one COMMAND_ID_TAG inside a START_TAGS/END_TAGS bracket.
    enum class AddResult {
        Added,      // new id appended
        Retransmit, // id already present with an equal payload; not appended
        Conflict    // id already present with a different payload; not appended
    };

    // Header-only so tests can use it without linking the log/LogFileManager deps.
    AddResult addTag(const TunnelProtocol::TagInfo_t& tagInfo)
    {
        for (const auto& existing : *this) {
            if (existing.id != tagInfo.id) {
                continue;
            }
            return sameTag(existing, tagInfo) ? AddResult::Retransmit : AddResult::Conflict;
        }
        push_back(tagInfo);
        return AddResult::Added;
    }

    // Field-wise rather than memcmp: unset priors arrive as NaN and the sender's
    // NaN bit pattern is not part of the contract.
    static bool sameTag(const TunnelProtocol::TagInfo_t& a, const TunnelProtocol::TagInfo_t& b)
    {
        return a.id                                      == b.id
            && a.frequency_hz                            == b.frequency_hz
            && a.pulse_width_msecs                       == b.pulse_width_msecs
            && a.intra_pulse1_msecs                      == b.intra_pulse1_msecs
            && a.intra_pulse2_msecs                      == b.intra_pulse2_msecs
            && a.intra_pulse_uncertainty_msecs           == b.intra_pulse_uncertainty_msecs
            && a.intra_pulse_jitter_msecs                == b.intra_pulse_jitter_msecs
            && a.k                                       == b.k
            && _sameDouble(a.false_alarm_probability,       b.false_alarm_probability)
            && a.channelizer_channel_number              == b.channelizer_channel_number
            && a.channelizer_channel_center_frequency_hz == b.channelizer_channel_center_frequency_hz
            && _sameDouble(a.ip1_mu,    b.ip1_mu)
            && _sameDouble(a.ip1_sigma, b.ip1_sigma)
            && _sameDouble(a.ip2_mu,    b.ip2_mu)
            && _sameDouble(a.ip2_sigma, b.ip2_sigma);
    }

    // UDP port a detector binds for its IQ stream. HF/simulator is a single
    // decimator channel (10000/10001), so the tag's channel number is irrelevant.
    static int detectorDataPort(const TunnelProtocol::TagInfo_t& tagInfo, bool isHFMode, bool secondaryChannel)
    {
        const int secondaryIncrement = secondaryChannel ? 1 : 0;
        return isHFMode ? 10000 + secondaryIncrement
                        : 20000 + (static_cast<int>(tagInfo.channelizer_channel_number) - 1) * 2 + secondaryIncrement;
    }

    struct PortCollision {
        uint32_t tagIdA;
        uint32_t tagIdB;
        int      port;
    };

    // First pair of tags whose primary detectors would bind the same port; the
    // detector binds without SO_REUSEPORT, so the second one dies at startup.
    std::optional<PortCollision> findPortCollision(bool isHFMode) const
    {
        for (size_t a = 0; a < size(); ++a) {
            const int portA = detectorDataPort((*this)[a], isHFMode, false);
            for (size_t b = a + 1; b < size(); ++b) {
                if (detectorDataPort((*this)[b], isHFMode, false) == portA) {
                    return PortCollision { (*this)[a].id, (*this)[b].id, portA };
                }
            }
        }
        return std::nullopt;
    }

    std::string detectorConfigFileName  (const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel) const;
    bool        writeDetectorConfigs    (bool isHFMode) const;
    std::string channelizerCommandLine  () const;

private:
    static bool _sameDouble(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }

    bool        _writeDetectorConfig    (const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel, bool isHFMode) const;
};
