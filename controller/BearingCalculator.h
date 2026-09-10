#pragma once

#include "AntennaPattern.h"

#include <cstdint>
#include <optional>
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
        bool        sighted;        // detector's fold search found this candidate on this slice
    };

    struct Result {
        uint32_t    tag_id;
        float       bearing_deg;    // NaN when the tag had no detections or no candidate passed the floor
        float       r_squared;      // confidence 0..1 (wire field name kept)
        uint32_t    n_valid_slices; // detected slices
        uint32_t    n_sighted_slices; // slices where the candidate was independently found, not just measured
        float       best_snr;
        uint8_t     candidate_id;   // lock candidate this result was fitted from
        uint32_t    n_candidates;   // candidates compared for this tag
        bool        rejected;       // best candidate fell below the confidence floor
    };

    // Fits with the given antenna's pattern; the confidence floor starts at
    // that pattern's tuned value.
    explicit BearingCalculator(const AntennaPattern& pattern = AntennaPatterns::ra2a());

    // signal_power is linear, noise-subtracted power (drives the fit);
    // snr_db is only reported back as Result::best_snr. candidate_id groups
    // measurements taken at competing detector locks.
    void addSlice(float heading_deg, double signal_power, uint32_t tag_id,
                  double snr_db, uint8_t candidate_id = 0, bool sighted = false);
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
    const AntennaPattern& antenna() const { return _pattern; }
    void reset();

    // Linear-scale pattern value at an angle offset from boresight (degrees).
    static double patternLinear(const AntennaPattern& pattern, double offsetDeg);
    double patternLinear(double offsetDeg) const { return patternLinear(_pattern, offsetDeg); }

    // Bearing of the first accepted result whose lock was sighted on exactly
    // one heading: it has no independent second look, so one more dwell
    // there separates a tag from a noise peak. nullopt when none qualifies.
    // One revisit per collection: with several such tags only the first gets it.
    static std::optional<float> revisitHeadingFor(const std::vector<Result>& results);

private:
    Result _solveForTag(uint32_t tag_id, const std::vector<SliceData>& slices) const;
    static void _fitAmplitude(const std::vector<double>& g, const std::vector<double>& p,
                              bool fitFloor, double& A, double& B);

    static constexpr double kScanStepDeg          = 0.5;
    static constexpr int    kMinDetectedForFloor  = 3;     // fewer → noise floor pinned at 0
    static constexpr double kCensorFraction       = 0.5;   // no-detection ⇒ power < this × weakest detection
    static constexpr double kSpanFloorFraction    = 0.005; // of Σpower², cost tolerance for the plausible φ set
    static constexpr double kFullDof              = 5.0;   // surplus observations for full confidence weight

    std::vector<SliceData> _slices;
    AntennaPattern _pattern;   // by value: callers may pass a temporary
    float _confidenceFloor;
};
