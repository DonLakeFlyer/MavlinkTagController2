#pragma once

#include <cstdint>
#include <vector>

class BearingCalculator {
public:
    struct SliceData {
        float       heading_deg;
        double      signal_power;
        uint32_t    tag_id;
        double      snr_db;
        bool        detected;
    };

    struct Result {
        uint32_t    tag_id;
        float       bearing_deg;    // NaN only when the tag had no detections
        float       r_squared;      // confidence 0..1 (wire field name kept)
        uint32_t    n_valid_slices; // detected slices
        float       best_snr;
    };

    // signal_power is linear, noise-subtracted power (drives the fit);
    // snr_db is only reported back as Result::best_snr.
    void addSlice(float heading_deg, double signal_power, uint32_t tag_id,
                  double snr_db);
    // Armed heading where the detector reported no pulse. Enters the fit as a
    // censored observation: predicted power must not exceed a fraction of the
    // weakest detected power in the rotation.
    void addNoDetection(float heading_deg, uint32_t tag_id);
    std::vector<Result> solve() const;
    void reset();

    // Exposed for test access — returns the linear-scale pattern value at an
    // arbitrary angle offset from boresight (degrees).
    static double patternLinear(double offsetDeg);

private:
    Result _solveForTag(uint32_t tag_id, const std::vector<SliceData>& slices) const;
    static void _fitAmplitude(const std::vector<double>& g, const std::vector<double>& p,
                              bool fitFloor, double& A, double& B);

    // RA-2AK measured antenna pattern in dB, normalized to 0 dB at boresight.
    // Eyeballed from the Telonics RA-2A reception radiation pattern polar plot.
    // 19 entries for 0–180° in 10° steps. Pattern is symmetric (mirrored for 180–360°).
    static constexpr int kPatternSize = 19;
    static const double kPatternDb[kPatternSize];

    static constexpr double kScanStepDeg          = 0.5;
    static constexpr int    kMinDetectedForFloor  = 3;     // fewer → noise floor pinned at 0
    static constexpr double kCensorFraction       = 0.5;   // no-detection ⇒ power < this × weakest detection
    static constexpr double kSpanFloorFraction    = 0.005; // of Σpower², cost tolerance for the plausible φ set
    static constexpr double kFullDof              = 5.0;   // surplus observations for full confidence weight

    std::vector<SliceData> _slices;
};
