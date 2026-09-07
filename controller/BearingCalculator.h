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
        uint8_t     candidate_id;   // detector lock candidate; no-detections are candidate-agnostic
    };

    struct Result {
        uint32_t    tag_id;
        float       bearing_deg;    // NaN when the tag had no detections or no candidate passed the floor
        float       r_squared;      // confidence 0..1 (wire field name kept)
        uint32_t    n_valid_slices; // detected slices
        float       best_snr;
        uint8_t     candidate_id;   // lock candidate this result was fitted from
        uint32_t    n_candidates;   // candidates compared for this tag
        bool        rejected;       // best candidate fell below the confidence floor
    };

    // Rotations whose best candidate scores below this report no bearing
    // (NaN) rather than a guess. Provisional: composite confidence for a
    // modelled false lock at 0-3 dB sits near 0.15-0.2 (fit ~0.4 x span);
    // a true lock on 8 headings scores ~0.7+. To be tuned by Monte-Carlo.
    static constexpr float kDefaultConfidenceFloor = 0.2f;

    // signal_power is linear, noise-subtracted power (drives the fit);
    // snr_db is only reported back as Result::best_snr. candidate_id groups
    // measurements taken at competing detector locks.
    void addSlice(float heading_deg, double signal_power, uint32_t tag_id,
                  double snr_db, uint8_t candidate_id = 0);
    // Armed heading where the detector reported no pulse. Enters the fit as a
    // censored observation: predicted power must not exceed a fraction of the
    // weakest detected power in the rotation. Applies to every candidate.
    void addNoDetection(float heading_deg, uint32_t tag_id);
    // One result per (tag, candidate), unselected and unfloored.
    std::vector<Result> solveCandidates() const;
    // One result per tag: the candidate with the highest confidence, with
    // bearing_deg set to NaN (rejected=true) when it is below the floor.
    std::vector<Result> solve() const;
    void setConfidenceFloor(float floor) { _confidenceFloor = floor; }
    float confidenceFloor() const { return _confidenceFloor; }
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
    float _confidenceFloor = kDefaultConfidenceFloor;
};
