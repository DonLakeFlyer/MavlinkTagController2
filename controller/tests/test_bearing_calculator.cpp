#include "AntennaPattern.h"
#include "BearingCalculator.h"
#include "TunnelProtocol.h"
#include "test_check.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

static constexpr float kBearingToleranceDeg = 5.0f;

static double ra2aPattern(double offsetDeg) {
    return BearingCalculator::patternLinear(AntennaPatterns::ra2a(), offsetDeg);
}

static void assertNear(float actual, float expected, float tolerance, const char* label) {
    // Handle wraparound for bearing
    float diff = std::fmod(std::fabs(actual - expected) + 360.0f, 360.0f);
    if (diff > 180.0f) diff = 360.0f - diff;
    if (diff > tolerance) {
        std::fprintf(stderr, "FAIL: %s — expected %.1f, got %.1f (diff %.1f > tolerance %.1f)\n",
                     label, expected, actual, diff, tolerance);
        std::exit(1);
    }
}

// ── Helper: generate synthetic SNR data for a given true bearing ────
// Uses the real RA-2AK antenna pattern via BearingCalculator::patternLinear().
// Model: SNR_i = amplitude * patternLinear(heading_i - trueBearing) + noiseFloor
static std::vector<std::pair<float, double>> generateSlices(
    float trueBearing, double amplitude, double noiseFloor, int nSlices,
    const AntennaPattern& pattern = AntennaPatterns::ra2a())
{
    std::vector<std::pair<float, double>> slices;
    float step = 360.0f / nSlices;
    for (int i = 0; i < nSlices; ++i) {
        float heading = std::fmod(i * step, 360.0f);
        double snr = amplitude * BearingCalculator::patternLinear(pattern, heading - trueBearing) + noiseFloor;
        slices.push_back({heading, snr});
    }
    return slices;
}

// ── Test: bearing at 0° with 8 slices (real flight config) ──────────
static void testBearingAtZero_8slices() {
    BearingCalculator calc;
    auto slices = generateSlices(0.0f, 30.0, 5.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 2, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 0.0f, kBearingToleranceDeg, "bearing at 0° (8 slices)");
    CHECK(results[0].r_squared > 0.9f);
    CHECK(results[0].n_valid_slices == 8);
    std::printf("PASS: testBearingAtZero_8slices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: bearing at 90° with 8 slices ──────────────────────────────
static void testBearingAt90_8slices() {
    BearingCalculator calc;
    auto slices = generateSlices(90.0f, 25.0, 8.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 3, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 90.0f, kBearingToleranceDeg, "bearing at 90° (8 slices)");
    CHECK(results[0].r_squared > 0.9f);
    std::printf("PASS: testBearingAt90_8slices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: bearing at 225° with 8 slices ─────────────────────────────
static void testBearingAt225_8slices() {
    BearingCalculator calc;
    auto slices = generateSlices(225.0f, 20.0, 10.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 4, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 225.0f, kBearingToleranceDeg, "bearing at 225° (8 slices)");
    CHECK(results[0].r_squared > 0.9f);
    std::printf("PASS: testBearingAt225_8slices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: bearing near 360° wraparound (350°) with 8 slices ────────
static void testBearingWraparound_8slices() {
    BearingCalculator calc;
    auto slices = generateSlices(350.0f, 28.0, 6.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 5, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 350.0f, 10.0f, "bearing wraparound 350° (8 slices)");
    CHECK(results[0].r_squared > 0.9f);
    std::printf("PASS: testBearingWraparound_8slices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: bearing between compass points (22° — not on a 45° grid) ─
static void testBearingOffGrid_8slices() {
    BearingCalculator calc;
    auto slices = generateSlices(22.0f, 30.0, 5.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 6, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 22.0f, 10.0f, "bearing at 22° off-grid (8 slices)");
    CHECK(results[0].r_squared > 0.9f);
    std::printf("PASS: testBearingOffGrid_8slices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: multiple tags in same solve ───────────────────────────────
static void testMultipleTags() {
    BearingCalculator calc;
    auto slicesA = generateSlices(45.0f, 30.0, 5.0, 8);
    auto slicesB = generateSlices(270.0f, 25.0, 8.0, 8);
    for (const auto& [hdg, snr] : slicesA) {
        calc.addSlice(hdg, snr, 10, 0.0);
    }
    for (const auto& [hdg, snr] : slicesB) {
        calc.addSlice(hdg, snr, 11, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 2);

    // Find results by tag
    const BearingCalculator::Result* r10 = nullptr;
    const BearingCalculator::Result* r11 = nullptr;
    for (const auto& r : results) {
        if (r.tag_id == 10) r10 = &r;
        if (r.tag_id == 11) r11 = &r;
    }
    CHECK(r10 && r11);
    assertNear(r10->bearing_deg, 45.0f, 10.0f, "multi-tag bearing 45°");
    assertNear(r11->bearing_deg, 270.0f, 10.0f, "multi-tag bearing 270°");
    std::printf("PASS: testMultipleTags (tag10=%.1f, tag11=%.1f)\n",
                r10->bearing_deg, r11->bearing_deg);
}

// ── Test: two detections still yield a bearing between them ─────────
static void testTwoSlices() {
    BearingCalculator calc;
    calc.setConfidenceFloor(0.0f);  // exercising the fit, not the rejection rule
    calc.addSlice(0.0f, 40.0, 2, 12.5);
    calc.addSlice(90.0f, 30.0, 2, 9.0);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    // Stronger slice at 0°, weaker at 90° → bearing leans toward 0° but
    // must sit between them; mirror solution behind the antenna is rejected.
    CHECK(results[0].bearing_deg > 0.0f && results[0].bearing_deg < 90.0f);
    // Exact fit, but only two observations for two parameters: low confidence.
    CHECK(results[0].r_squared > 0.0f && results[0].r_squared < 0.5f);
    CHECK(results[0].n_valid_slices == 2);
    CHECK(results[0].best_snr == 12.5f);
    CHECK(!results[0].rejected);
    std::printf("PASS: testTwoSlices (bearing=%.1f, conf=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: single slice returns that heading with zero confidence ───
static void testSingleSlice() {
    BearingCalculator calc;
    calc.setConfidenceFloor(0.0f);
    calc.addSlice(123.0f, 35.0, 7, 11.0);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 123.0f, 0.01f, "single slice heading");
    CHECK(results[0].r_squared == 0.0f);
    CHECK(results[0].n_valid_slices == 1);
    CHECK(results[0].best_snr == 11.0f);
    std::printf("PASS: testSingleSlice\n");
}

// ── Test: with the default floor a lone detection is not a bearing ────
static void testSingleSliceRejectedByFloor() {
    BearingCalculator calc;
    calc.addSlice(123.0f, 35.0, 7, 11.0);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].rejected);
    CHECK(std::isnan(results[0].bearing_deg));
    CHECK(results[0].r_squared == 0.0f);
    CHECK(results[0].n_valid_slices == 1);   // still reported so the GCS can show what was seen
    std::printf("PASS: testSingleSliceRejectedByFloor\n");
}

// ── Test: competing lock candidates — pattern-shaped one wins ─────────
// Candidate 0 (the detector's provisional lock) is a noise lock: flat-ish
// powers with no pattern structure. Candidate 1 traces the antenna pattern
// around the rotation. Selection must pick candidate 1 even though the
// detector reported 0 first.
static void testCandidateSelection() {
    BearingCalculator calc;
    const double noiseLock[8] = {3.1, 2.7, 3.4, 2.9, 3.3, 2.6, 3.0, 3.2};
    for (int i = 0; i < 8; ++i) {
        calc.addSlice(static_cast<float>(i * 45), noiseLock[i], 2, 3.0, 0);
    }
    auto slices = generateSlices(225.0f, 30.0, 2.0, 8);
    for (const auto& [hdg, power] : slices) {
        calc.addSlice(hdg, power, 2, 9.0, 1);
    }

    auto candidates = calc.solveCandidates();
    CHECK(candidates.size() == 2);
    for (const auto& c : candidates) {
        CHECK(c.tag_id == 2);
        CHECK(c.n_candidates == 2);
    }

    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].candidate_id == 1);
    CHECK(results[0].n_candidates == 2);
    CHECK(!results[0].rejected);
    assertNear(results[0].bearing_deg, 225.0f, kBearingToleranceDeg, "selected candidate bearing");
    CHECK(results[0].r_squared > 0.9f);
    CHECK(results[0].best_snr == 9.0f);
    std::printf("PASS: testCandidateSelection (candidate=%u, bearing=%.1f, conf=%.3f)\n",
                results[0].candidate_id, results[0].bearing_deg, results[0].r_squared);
}

// ── Test: no-detection headings are censored observations for every candidate ──
static void testNoDetectionsSharedAcrossCandidates() {
    BearingCalculator calc;
    // Both candidates saw a pulse only at 0°; six other headings were empty.
    calc.addSlice(0.0f, 32.8, 2, 0.0, 0);
    calc.addSlice(0.0f, 20.0, 2, 0.0, 1);
    for (int i = 1; i < 8; ++i) {
        calc.addNoDetection(static_cast<float>(i * 45), 2);
    }
    auto candidates = calc.solveCandidates();
    CHECK(candidates.size() == 2);
    for (const auto& c : candidates) {
        // Without the censored headings a lone detection scores 0.
        CHECK(c.r_squared > 0.3f);
        assertNear(c.bearing_deg, 0.0f, 10.0f, "censored shared across candidates");
    }
    std::printf("PASS: testNoDetectionsSharedAcrossCandidates\n");
}

// ── Test: every candidate below the floor → no bearing, best one reported ──
static void testAllCandidatesRejected() {
    BearingCalculator calc;
    calc.setConfidenceFloor(0.95f);
    auto good = generateSlices(90.0f, 30.0, 5.0, 8);
    for (size_t i = 0; i < good.size(); ++i) {
        // ±2 dB alternating noise keeps confidence below 0.95
        calc.addSlice(good[i].first, good[i].second + ((i % 2) ? -2.0 : 2.0), 3, 0.0, 0);
        calc.addSlice(good[i].first, 3.0 + 0.1 * i, 3, 0.0, 1);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].rejected);
    CHECK(std::isnan(results[0].bearing_deg));
    CHECK(results[0].candidate_id == 0);   // best of the bad set is still identified
    CHECK(results[0].r_squared > 0.5f && results[0].r_squared < 0.95f);
    CHECK(results[0].n_valid_slices == 8);
    std::printf("PASS: testAllCandidatesRejected (conf=%.3f)\n", results[0].r_squared);
}

// ── Test: single candidate keeps legacy behaviour (candidate 0, not rejected) ──
static void testSingleCandidateDefaults() {
    BearingCalculator calc;
    auto slices = generateSlices(0.0f, 30.0, 5.0, 8);
    for (const auto& [hdg, power] : slices) {
        calc.addSlice(hdg, power, 2, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].candidate_id == 0);
    CHECK(results[0].n_candidates == 1);
    CHECK(!results[0].rejected);
    std::printf("PASS: testSingleCandidateDefaults\n");
}

// ── Test: one detection plus no-detections everywhere else ──────────
// The marginal-run case: tag locked on one heading only. Censored headings
// must pin the bearing to that heading with real confidence.
static void testSingleDetectionWithCensored() {
    BearingCalculator calc;
    calc.addSlice(0.0f, 32.8, 2, 0.0);
    for (int i = 1; i < 8; ++i) {
        calc.addNoDetection(static_cast<float>(i * 45), 2);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 0.0f, 10.0f, "single detection + censored");
    CHECK(results[0].r_squared > 0.3f);
    CHECK(results[0].n_valid_slices == 1);
    std::printf("PASS: testSingleDetectionWithCensored (bearing=%.1f, conf=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: the observed marginal run (32.8 at N, 1.2 at NW, rest none) ──
static void testMarginalRun() {
    BearingCalculator calc;
    calc.addSlice(0.0f, 32.8, 2, 0.0);
    calc.addSlice(315.0f, 1.2, 2, 0.0);
    for (int i = 1; i < 7; ++i) {
        calc.addNoDetection(static_cast<float>(i * 45), 2);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 0.0f, 20.0f, "marginal run bearing");
    CHECK(results[0].r_squared > 0.2f);
    CHECK(results[0].n_valid_slices == 2);
    std::printf("PASS: testMarginalRun (bearing=%.1f, conf=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: only no-detections → NaN bearing, zero confidence ────────
static void testOnlyCensored() {
    BearingCalculator calc;
    for (int i = 0; i < 8; ++i) {
        calc.addNoDetection(static_cast<float>(i * 45), 4);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(std::isnan(results[0].bearing_deg));
    CHECK(results[0].r_squared == 0.0f);
    CHECK(results[0].n_valid_slices == 0);
    std::printf("PASS: testOnlyCensored\n");
}

// ── Test: empty calculator returns no results ───────────────────────
static void testEmpty() {
    BearingCalculator calc;
    auto results = calc.solve();
    CHECK(results.empty());
    std::printf("PASS: testEmpty\n");
}

// ── Test: reset clears state ────────────────────────────────────────
static void testReset() {
    BearingCalculator calc;
    auto slices = generateSlices(180.0f, 25.0, 5.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 2, 0.0);
    }
    calc.reset();
    auto results = calc.solve();
    CHECK(results.empty());
    std::printf("PASS: testReset\n");
}

// ── Test: best_snr reports the max snr_db, independent of signal_power ──
static void testBestSnr() {
    BearingCalculator calc;
    // Strongest power carries a lower SNR than a weaker slice; best_snr must
    // track the dB field, not the power field.
    calc.addSlice(0.0f, 40.0, 2, 8.0);
    calc.addSlice(90.0f, 25.0, 2, 14.5);
    calc.addSlice(180.0f, 20.0, 2, 6.0);
    calc.addSlice(270.0f, 28.0, 2, 9.0);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].best_snr == 14.5f);
    std::printf("PASS: testBestSnr (best_snr=%.1f)\n", results[0].best_snr);
}

// ── Test: steep side-lobe drop-off must not drive the noise floor negative ──
// Unconstrained OLS on this data gives B < 0, which predicts ≤0 power at the
// censored back-lobe headings and thereby switches off their penalty
// (r_squared inflated to ~0.99). With B pinned at 0 the censored headings
// still cost, so confidence must stay below that.
static void testNegativeFloorRejected() {
    BearingCalculator calc;
    calc.addSlice(0.0f, 30.0, 2, 0.0);
    calc.addSlice(45.0f, 8.0, 2, 0.0);
    calc.addSlice(315.0f, 0.5, 2, 0.0);
    for (int h : {90, 135, 180, 225, 270}) {
        calc.addNoDetection(static_cast<float>(h), 2);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 0.0f, 10.0f, "negative floor rejected");
    CHECK(results[0].r_squared > 0.5f && results[0].r_squared < 0.9f);
    std::printf("PASS: testNegativeFloorRejected (bearing=%.1f, conf=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: noisy data still converges within tolerance ───────────────
static void testNoisyData() {
    BearingCalculator calc;
    float trueBearing = 135.0f;
    auto slices = generateSlices(trueBearing, 30.0, 5.0, 8);

    // Add systematic noise: ±2.0 dB alternating
    for (size_t i = 0; i < slices.size(); ++i) {
        double noise = (i % 2 == 0) ? 2.0 : -2.0;
        calc.addSlice(slices[i].first, slices[i].second + noise, 6, 0.0);
    }

    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, trueBearing, 15.0f, "noisy bearing at 135°");
    CHECK(results[0].r_squared > 0.8f);
    std::printf("PASS: testNoisyData (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: pattern symmetry — front and back lobes ───────────────────
static void testPatternSymmetry() {
    // RA-2AK pattern should be symmetric: pattern(+30°) == pattern(-30°)
    CHECK(std::fabs(ra2aPattern(30.0) - ra2aPattern(-30.0)) < 1e-10);

    // Boresight should be 1.0 (0 dB)
    double p0 = ra2aPattern(0.0);
    CHECK(std::fabs(p0 - 1.0) < 1e-10);

    // Back lobe at 180° should be ~0.1 (-10 dB)
    double p180 = ra2aPattern(180.0);
    CHECK(std::fabs(p180 - 0.1) < 0.001);

    // Deep null near 90° should be very low
    double p90 = ra2aPattern(90.0);
    CHECK(p90 < 0.01);  // -27.5 dB ≈ 0.00178

    std::printf("PASS: testPatternSymmetry (0°=%.4f, 90°=%.5f, 180°=%.4f)\n", p0, p90, p180);
}

// ── Test: antenna registry ─────────────────────────────────────────────────
static void testAntennaRegistry() {
    const AntennaPattern& ra2a  = AntennaPatterns::byId(ANTENNA_ID_RA2A);
    const AntennaPattern& ra23k = AntennaPatterns::byId(ANTENNA_ID_RA23K);
    CHECK(ra2a.id == ANTENNA_ID_RA2A);
    CHECK(ra23k.id == ANTENNA_ID_RA23K);
    CHECK(&ra2a != &ra23k);
    CHECK(AntennaPatterns::isKnown(ANTENNA_ID_RA2A));
    CHECK(AntennaPatterns::isKnown(ANTENNA_ID_RA23K));
    CHECK(!AntennaPatterns::isKnown(999));
    bool threw = false;
    try {
        AntennaPatterns::byId(999);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    // Both tables are boresight-normalised and symmetric.
    for (const AntennaPattern* p : {&ra2a, &ra23k}) {
        CHECK(std::fabs(BearingCalculator::patternLinear(*p, 0.0) - 1.0) < 1e-10);
        CHECK(std::fabs(BearingCalculator::patternLinear(*p, 60.0)
                        - BearingCalculator::patternLinear(*p, 300.0)) < 1e-10);
        CHECK(p->confidenceFloor > 0.0f && p->confidenceFloor < 1.0f);
    }
    // The 3-element has a shallower side null than the 2-element.
    CHECK(BearingCalculator::patternLinear(ra23k, 90.0) > BearingCalculator::patternLinear(ra2a, 90.0));

    // The calculator fits with the pattern it was built with.
    BearingCalculator calc(ra23k);
    CHECK(calc.confidenceFloor() == ra23k.confidenceFloor);
    for (const auto& [hdg, snr] : generateSlices(135.0f, 30.0, 5.0, 8, ra23k)) {
        calc.addSlice(hdg, snr, 7, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 135.0f, kBearingToleranceDeg, "RA-23K bearing at 135°");
    CHECK(results[0].r_squared > 0.9f);
    std::printf("PASS: testAntennaRegistry (RA-23K bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: sighted slices are counted per candidate ─────────────────────
static void testSightedSliceCount() {
    BearingCalculator calc;
    calc.setConfidenceFloor(0.0f);
    auto slices = generateSlices(45.0f, 30.0, 0.0, 8);
    for (size_t i = 0; i < slices.size(); ++i) {
        // Fold search found candidate 0 on the two front-lobe headings only;
        // the rest are fixed-offset measurements at that lock.
        const bool sighted = slices[i].first == 45.0f || slices[i].first == 90.0f;
        calc.addSlice(slices[i].first, slices[i].second, 2, 0.0, 0, sighted);
    }
    // A competing candidate measured everywhere but sighted once.
    for (size_t i = 0; i < slices.size(); ++i) {
        calc.addSlice(slices[i].first, 0.1 * slices[i].second, 2, 0.0, 1, slices[i].first == 0.0f);
    }
    auto results = calc.solveCandidates();
    CHECK(results.size() == 2);
    for (const auto& r : results) {
        CHECK(r.n_valid_slices == 8);
        CHECK(r.n_sighted_slices == (r.candidate_id == 0 ? 2u : 1u));
    }
    // No-detection headings never count as sightings.
    BearingCalculator lone;
    lone.setConfidenceFloor(0.0f);
    lone.addSlice(45.0f, 30.0, 3, 0.0, 0, true);
    lone.addNoDetection(225.0f, 3);
    auto loneResults = lone.solve();
    CHECK(loneResults.size() == 1);
    CHECK(loneResults[0].n_sighted_slices == 1);
    std::printf("PASS: testSightedSliceCount\n");
}

// ── Test: revisit is requested only for an accepted single-sighting winner ─
static void testRevisitHeadingFor() {
    auto result = [](uint32_t tag, float bearing, uint32_t sighted, bool rejected) {
        BearingCalculator::Result r {};
        r.tag_id = tag;
        r.bearing_deg = bearing;
        r.r_squared = 0.9f;
        r.n_valid_slices = 8;
        r.n_sighted_slices = sighted;
        r.rejected = rejected;
        return r;
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();

    CHECK(!BearingCalculator::revisitHeadingFor({}).has_value());
    CHECK(!BearingCalculator::revisitHeadingFor({result(2, 45.0f, 1, true)}).has_value());
    CHECK(!BearingCalculator::revisitHeadingFor({result(2, nan, 1, false)}).has_value());
    CHECK(!BearingCalculator::revisitHeadingFor({result(2, 45.0f, 2, false)}).has_value());
    CHECK(!BearingCalculator::revisitHeadingFor({result(2, 45.0f, 0, false)}).has_value());

    auto one = BearingCalculator::revisitHeadingFor({result(2, 45.0f, 1, false)});
    CHECK(one.has_value() && *one == 45.0f);

    // Confirmed first tag, unconfirmed second: the second one drives the revisit.
    auto second = BearingCalculator::revisitHeadingFor(
        {result(2, 45.0f, 3, false), result(3, 270.0f, 1, false)});
    CHECK(second.has_value() && *second == 270.0f);
    std::printf("PASS: testRevisitHeadingFor\n");
}

// ── Test: 16-slice rotation ─────────────────────────────────────────
static void testBearing_16slices() {
    BearingCalculator calc;
    auto slices = generateSlices(160.0f, 30.0, 5.0, 16);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 9, 0.0);
    }
    auto results = calc.solve();
    CHECK(results.size() == 1);
    assertNear(results[0].bearing_deg, 160.0f, kBearingToleranceDeg, "bearing at 160° (16 slices)");
    CHECK(results[0].r_squared > 0.95f);
    std::printf("PASS: testBearing_16slices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

int main() {
    testEmpty();
    testSingleSlice();
    testSingleSliceRejectedByFloor();
    testTwoSlices();
    testSingleDetectionWithCensored();
    testMarginalRun();
    testOnlyCensored();
    testReset();
    testBestSnr();
    testNegativeFloorRejected();
    testCandidateSelection();
    testNoDetectionsSharedAcrossCandidates();
    testAllCandidatesRejected();
    testSingleCandidateDefaults();
    testPatternSymmetry();
    testAntennaRegistry();
    testSightedSliceCount();
    testRevisitHeadingFor();
    testBearingAtZero_8slices();
    testBearingAt90_8slices();
    testBearingAt225_8slices();
    testBearingWraparound_8slices();
    testBearingOffGrid_8slices();
    testMultipleTags();
    testNoisyData();
    testBearing_16slices();
    std::printf("\nAll BearingCalculator tests passed.\n");
    return 0;
}
