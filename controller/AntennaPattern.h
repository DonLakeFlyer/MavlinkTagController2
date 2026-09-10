#pragma once

#include <cstdint>

// Reception pattern of one directional antenna, used by the bearing fit.
struct AntennaPattern {
    static constexpr int kSize = 19;    // 0..180 deg in 10 deg steps; mirrored for 180..360

    uint32_t    id;                     // TunnelProtocol ANTENNA_ID_*
    const char* name;
    double      patternDb[kSize];       // normalised to 0 dB at boresight
    // Rotations whose best candidate scores below this report no bearing.
    // Pattern-specific: a flatter rear lobe lets noise fit "something" more
    // often. Tuned per antenna by tools/confidence_floor_montecarlo.
    float       confidenceFloor;
};

namespace AntennaPatterns {

const AntennaPattern& ra2a();
const AntennaPattern& ra23k();
bool isKnown(uint32_t antennaId);
// Throws std::invalid_argument for an unknown id; StartCollection rejects
// those before any fit runs.
const AntennaPattern& byId(uint32_t antennaId);

}
