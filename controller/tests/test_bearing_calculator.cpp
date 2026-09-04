#include "BearingCalculator.h"
#include "test_check.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

static constexpr float kBearingToleranceDeg = 5.0f;

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
    float trueBearing, double amplitude, double noiseFloor, int nSlices)
{
    std::vector<std::pair<float, double>> slices;
    float step = 360.0f / nSlices;
    for (int i = 0; i < nSlices; ++i) {
        float heading = std::fmod(i * step, 360.0f);
        double snr = amplitude * BearingCalculator::patternLinear(heading - trueBearing) + noiseFloor;
        slices.push_back({heading, snr});
    }
    return slices;
}

// ── Test: bearing at 0° with 8 slices (real flight config) ──────────
static void testBearingAtZero_8slices() {
    BearingCalculator calc;
    auto slices = generateSlices(0.0f, 30.0, 5.0, 8);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 2);
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
        calc.addSlice(hdg, snr, 3);
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
        calc.addSlice(hdg, snr, 4);
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
        calc.addSlice(hdg, snr, 5);
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
        calc.addSlice(hdg, snr, 6);
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
        calc.addSlice(hdg, snr, 10);
    }
    for (const auto& [hdg, snr] : slicesB) {
        calc.addSlice(hdg, snr, 11);
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

// ── Test: too few slices returns best-heading fallback ───────────────
static void testTooFewSlices() {
    BearingCalculator calc;
    // Only 2 slices — below kMinSlicesForFit (3)
    calc.addSlice(0.0f, 40.0, 2);
    calc.addSlice(90.0f, 30.0, 2);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].bearing_deg == 0.0f);  // heading of best SNR
    CHECK(results[0].r_squared == 0.0f);
    CHECK(results[0].n_valid_slices == 2);
    CHECK(results[0].best_snr == 40.0f);
    std::printf("PASS: testTooFewSlices (bearing=%.1f, R²=%.3f)\n",
                results[0].bearing_deg, results[0].r_squared);
}

// ── Test: single slice returns that heading ──────────────────────────
static void testSingleSlice() {
    BearingCalculator calc;
    calc.addSlice(123.0f, 35.0, 7);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].bearing_deg == 123.0f);
    CHECK(results[0].r_squared == 0.0f);
    CHECK(results[0].n_valid_slices == 1);
    CHECK(results[0].best_snr == 35.0f);
    std::printf("PASS: testSingleSlice\n");
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
        calc.addSlice(hdg, snr, 2);
    }
    calc.reset();
    auto results = calc.solve();
    CHECK(results.empty());
    std::printf("PASS: testReset\n");
}

// ── Test: best_snr field is correct ─────────────────────────────────
static void testBestSnr() {
    BearingCalculator calc;
    calc.addSlice(0.0f, 40.0, 2);
    calc.addSlice(90.0f, 25.0, 2);
    calc.addSlice(180.0f, 20.0, 2);
    calc.addSlice(270.0f, 28.0, 2);
    auto results = calc.solve();
    CHECK(results.size() == 1);
    CHECK(results[0].best_snr == 40.0f);
    std::printf("PASS: testBestSnr (best_snr=%.1f)\n", results[0].best_snr);
}

// ── Test: noisy data still converges within tolerance ───────────────
static void testNoisyData() {
    BearingCalculator calc;
    float trueBearing = 135.0f;
    auto slices = generateSlices(trueBearing, 30.0, 5.0, 8);

    // Add systematic noise: ±2.0 dB alternating
    for (size_t i = 0; i < slices.size(); ++i) {
        double noise = (i % 2 == 0) ? 2.0 : -2.0;
        calc.addSlice(slices[i].first, slices[i].second + noise, 6);
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
    CHECK(std::fabs(BearingCalculator::patternLinear(30.0) - BearingCalculator::patternLinear(-30.0)) < 1e-10);

    // Boresight should be 1.0 (0 dB)
    double p0 = BearingCalculator::patternLinear(0.0);
    CHECK(std::fabs(p0 - 1.0) < 1e-10);

    // Back lobe at 180° should be ~0.1 (-10 dB)
    double p180 = BearingCalculator::patternLinear(180.0);
    CHECK(std::fabs(p180 - 0.1) < 0.001);

    // Deep null near 90° should be very low
    double p90 = BearingCalculator::patternLinear(90.0);
    CHECK(p90 < 0.01);  // -27.5 dB ≈ 0.00178

    std::printf("PASS: testPatternSymmetry (0°=%.4f, 90°=%.5f, 180°=%.4f)\n", p0, p90, p180);
}

// ── Test: 16-slice rotation ─────────────────────────────────────────
static void testBearing_16slices() {
    BearingCalculator calc;
    auto slices = generateSlices(160.0f, 30.0, 5.0, 16);
    for (const auto& [hdg, snr] : slices) {
        calc.addSlice(hdg, snr, 9);
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
    testTooFewSlices();
    testReset();
    testBestSnr();
    testPatternSymmetry();
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
