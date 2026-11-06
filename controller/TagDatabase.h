#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "TunnelProtocol.h"
#include "WireText.h"

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
    // NaN bit pattern is not part of the contract. The display names are not
    // identity, so a rename re-upload is a retransmit.
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

    // Log suffix naming a detector id: ` name:"Tag"`, plus ` rate:"Resting"` for the
    // uavrt secondary detector (id+1). Empty when the tag is unknown or unnamed.
    std::string nameSuffix(uint32_t detectorId) const
    {
        bool secondary = false;
        const TunnelProtocol::TagInfo_t* tag = _findDetectorTag(detectorId, secondary);
        if (!tag) {
            return {};
        }
        const std::string name = wireText(tag->name);
        if (name.empty()) {
            return {};
        }
        std::string suffix = " name:\"" + name + "\"";
        if (secondary) {
            suffix += _rateSuffix(wireText(tag->rate2_name));
        }
        return suffix;
    }

    // Log suffix for a Python detector rate_state (kRateStateXxx): ` rate:"Moving"`,
    // ` rate:"Moving>Resting"` for a switch. Empty when the rate names are unset.
    std::string rateSuffix(uint32_t tagId, uint8_t rateState) const
    {
        bool secondary = false;
        const TunnelProtocol::TagInfo_t* tag = _findDetectorTag(tagId, secondary);
        if (!tag || secondary) {
            return {};
        }
        const std::string a = wireText(tag->rate1_name);
        const std::string b = wireText(tag->rate2_name);
        switch (rateState) {
        case TunnelProtocol::kRateStateA:    return _rateSuffix(a);
        case TunnelProtocol::kRateStateB:    return _rateSuffix(b);
        case TunnelProtocol::kRateStateAToB: return a.empty() || b.empty() ? std::string() : _rateSuffix(a + ">" + b);
        case TunnelProtocol::kRateStateBToA: return a.empty() || b.empty() ? std::string() : _rateSuffix(b + ">" + a);
        default:                             return {};
        }
    }

    std::string detectorConfigFileName  (const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel) const;
    bool        writeDetectorConfigs    (bool isHFMode) const;
    std::string channelizerCommandLine  () const;

private:
    static bool _sameDouble(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }

    static std::string _rateSuffix(const std::string& rate)
    {
        return rate.empty() ? std::string() : " rate:\"" + rate + "\"";
    }

    // An exact id match wins over a dual-rate tag's secondary id (id+1).
    const TunnelProtocol::TagInfo_t* _findDetectorTag(uint32_t detectorId, bool& secondary) const
    {
        secondary = false;
        for (const auto& tag : *this) {
            if (tag.id == detectorId) {
                return &tag;
            }
        }
        for (const auto& tag : *this) {
            if (tag.intra_pulse2_msecs != 0 && tag.id + 1 == detectorId) {
                secondary = true;
                return &tag;
            }
        }
        return nullptr;
    }

    bool        _writeDetectorConfig    (const TunnelProtocol::TagInfo_t& tagInfo, bool secondaryChannel, bool isHFMode) const;
};
