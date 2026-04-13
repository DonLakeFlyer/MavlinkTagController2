#pragma once

#include <cstdint>
#include <vector>

class BearingCalculator {
public:
    struct SliceData {
        float       heading_deg;
        double      snr_db;
        uint32_t    tag_id;
    };

    struct Result {
        uint32_t    tag_id;
        float       bearing_deg;
        float       r_squared;
        uint32_t    n_valid_slices;
        float       best_snr;
    };

    void addSlice(float heading_deg, double snr_db, uint32_t tag_id);
    std::vector<Result> solve() const;
    void reset();

    // Exposed for test access — returns the linear-scale pattern value at an
    // arbitrary angle offset from boresight (degrees).
    static double patternLinear(double offsetDeg);

private:
    Result _solveForTag(uint32_t tag_id, const std::vector<SliceData>& slices) const;

    // RA-2AK measured antenna pattern in dB, normalized to 0 dB at boresight.
    // Eyeballed from the Telonics RA-2A reception radiation pattern polar plot.
    // 19 entries for 0–180° in 10° steps. Pattern is symmetric (mirrored for 180–360°).
    static constexpr int kPatternSize = 19;
    static const double kPatternDb[kPatternSize];

    static constexpr int    kMaxIterations  = 100;
    static constexpr double kConvergenceEps = 1e-6;
    static constexpr double kNumDiffStep    = 0.5;   // degrees for numerical derivative
    static constexpr double kMaxPhiStep     = 15.0;  // max bearing change per iteration
    static constexpr int    kMinSlicesForFit = 3;

    std::vector<SliceData> _slices;
};
