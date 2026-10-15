#include "AntennaPattern.h"

#include "TunnelProtocol.h"

#include <stdexcept>
#include <string>

namespace {

// Tuned RA-2AHS (147 MHz) as flown on the vehicle. Measured on the 2026-10-15
// manual rotation (collar 181.5 deg true, 19 deg below the horizon) at
// 0/45/90/135/180 deg off boresight, +/-1 dB; values between are interpolated.
// The sides are floor-limited by the airframe and elevation. One flight, one
// elevation: refine with further rotations (docs/proposals/FIELD_TEST_PLAN.md).
// Vendor free-space plot: Antennas/RA-2AHS-tuned.jpg.
const AntennaPattern kRa2a = {
    ANTENNA_ID_RA2A,
    "RA-2A-tuned-147",
    {
        //  0      10     20     30     40     50     60     70     80     90
         0.0,   0.0,   0.0,  -1.0,  -2.8,  -8.5, -12.5, -16.5, -20.5, -24.2,
        // 100    110    120    130    140    150    160    170    180
       -23.5, -22.0, -20.5, -19.0, -17.9, -15.0, -11.5,  -8.0,  -6.4
    },
    0.2f,
};

// Eyeballed from the Telonics RA-23 reception radiation pattern polar plot
// (Antennas/RA-23K.jpg). Its radial scale is non-linear: 2 dB rings out to
// 10 dB, then 20/30/50 compressed toward the centre; expect +/-1.5 dB.
// Source: datasheet. Replace in place once measured installed (see kRa2a).
const AntennaPattern kRa23k = {
    ANTENNA_ID_RA23K,
    "RA-23K",
    {
        //  0      10     20     30     40     50     60     70     80     90
         0.0,  -0.5,  -1.0,  -2.0,  -3.0,  -5.0,  -8.0, -10.5, -16.0, -18.0,
        // 100    110    120    130    140    150    160    170    180
       -17.5, -16.0, -14.0, -13.5, -11.5, -10.5, -10.0,  -9.5, -10.0
    },
    0.2f,
};

}

namespace AntennaPatterns {

const AntennaPattern& ra2a()  { return kRa2a; }
const AntennaPattern& ra23k() { return kRa23k; }

bool isKnown(uint32_t antennaId)
{
    return antennaId == ANTENNA_ID_RA2A || antennaId == ANTENNA_ID_RA23K;
}

const AntennaPattern& byId(uint32_t antennaId)
{
    switch (antennaId) {
    case ANTENNA_ID_RA2A:  return kRa2a;
    case ANTENNA_ID_RA23K: return kRa23k;
    }
    throw std::invalid_argument("unknown antenna_id " + std::to_string(antennaId));
}

}
