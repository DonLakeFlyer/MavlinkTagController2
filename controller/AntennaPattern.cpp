#include "AntennaPattern.h"

#include "TunnelProtocol.h"

#include <stdexcept>
#include <string>

namespace {

// Eyeballed (+/-1.5 dB) from the Telonics tuned-antenna polar plot
// (Antennas/RA-2AHS-tuned.jpg): tuned for 147 MHz, linear 10 dB rings,
// 0 dB = front peak. Side nulls end at the plot's centre circle (~-36 dB),
// so 90/100 deg are a floor.
// Free-space plot; the installed pattern with the airframe will differ.
// Replace in place with the pattern measured on the calibration flight
// (docs/proposals/FIELD_TEST_PLAN.md).
const AntennaPattern kRa2a = {
    ANTENNA_ID_RA2A,
    "RA-2A-tuned-147",
    {
        //  0      10     20     30     40     50     60     70     80     90
         0.0,   0.0,   0.0,  -0.5,  -2.0,  -5.5, -10.5, -14.0, -23.5, -32.0,
        // 100    110    120    130    140    150    160    170    180
       -30.0, -19.5, -13.5, -12.5, -11.5, -11.0, -10.0, -10.0, -10.0
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
