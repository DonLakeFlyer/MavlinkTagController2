#include "BearingCalculator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

// RA-2AK measured antenna pattern in dB, normalized to 0 dB at boresight.
// Eyeballed from the Telonics RA-2A reception radiation pattern polar plot.
// 10° steps from 0° (front) to 180° (back). Pattern is symmetric so we mirror for 180-360°.
const double BearingCalculator::kPatternDb[BearingCalculator::kPatternSize] = {
    //  0°     10°     20°     30°     40°     50°     60°     70°     80°     90°
     0.0,    0.0,   -0.5,   -1.0,   -2.5,   -5.0,  -10.5,  -14.5,  -20.5,  -27.5,
    // 100°   110°   120°   130°   140°   150°   160°   170°   180°
   -20.0,  -17.5,  -14.5,  -12.5,  -13.5,  -10.5,  -10.5,  -10.0,  -10.0
};

void BearingCalculator::addSlice(float heading_deg, double snr_db, uint32_t tag_id)
{
    _slices.push_back({heading_deg, snr_db, tag_id});
}

void BearingCalculator::reset()
{
    _slices.clear();
}

// Interpolate the measured pattern at an arbitrary angle offset from boresight (degrees).
// Returns the pattern value in linear (power) scale, normalized so boresight = 1.0.
double BearingCalculator::patternLinear(double offsetDeg)
{
    // Normalize to 0-360 then fold to 0-180 (symmetric)
    double angle = std::fmod(offsetDeg, 360.0);
    if (angle < 0.0) angle += 360.0;
    if (angle > 180.0) angle = 360.0 - angle;

    // Interpolate in the 10° step table
    const double indexF = angle / 10.0;
    const int idx0 = static_cast<int>(indexF);
    const int idx1 = std::min(idx0 + 1, kPatternSize - 1);
    const double frac = indexF - idx0;

    const double db = kPatternDb[idx0] * (1.0 - frac) + kPatternDb[idx1] * frac;
    return std::pow(10.0, db / 10.0);
}

std::vector<BearingCalculator::Result> BearingCalculator::solve() const
{
    // Group slices by tag_id
    std::map<uint32_t, std::vector<SliceData>> tagGroups;
    for (const auto& s : _slices) {
        tagGroups[s.tag_id].push_back(s);
    }

    std::vector<Result> results;
    for (const auto& [tagId, slices] : tagGroups) {
        results.push_back(_solveForTag(tagId, slices));
    }
    return results;
}

BearingCalculator::Result BearingCalculator::_solveForTag(uint32_t tag_id, const std::vector<SliceData>& slices) const
{
    Result result {};
    result.tag_id = tag_id;
    result.n_valid_slices = static_cast<uint32_t>(slices.size());

    // Find best SNR
    double bestSnr = -1e9;
    float bestHeading = 0;
    for (const auto& s : slices) {
        if (s.snr_db > bestSnr) {
            bestSnr = s.snr_db;
            bestHeading = s.heading_deg;
        }
    }
    result.best_snr = static_cast<float>(bestSnr);

    if (static_cast<int>(slices.size()) < kMinSlicesForFit) {
        // Not enough data for a fit — return heading of best SNR
        result.bearing_deg = bestHeading;
        result.r_squared = 0.0f;
        return result;
    }

    const int N = static_cast<int>(slices.size());

    // Collect heading and SNR arrays
    std::vector<double> headings(N);
    std::vector<double> snrValues(N);
    for (int i = 0; i < N; ++i) {
        headings[i]  = slices[i].heading_deg;
        snrValues[i] = slices[i].snr_db;
    }

    // Model: SNR(θ) = A · patternLinear(θ - φ) + B
    // Three fitted parameters: φ (bearing), A (amplitude), B (noise floor)

    // Initial estimates for A and B from data range
    double maxVal = *std::max_element(snrValues.begin(), snrValues.end());
    double minVal = *std::min_element(snrValues.begin(), snrValues.end());
    double A = maxVal - minVal;
    double B = minVal;

    if (A <= 0.0) {
        // No SNR contrast — return heading of best SNR
        result.bearing_deg = bestHeading;
        result.r_squared = 0.0f;
        return result;
    }

    // Brute-force scan for best initial phi at 1° resolution.
    // The RA-2AK pattern has sharp nulls, so the LM solver can get stuck
    // at a local minimum if started at the nearest slice heading.
    double phi = 0.0;
    {
        double bestCost = std::numeric_limits<double>::max();
        for (int deg = 0; deg < 360; ++deg) {
            const double testPhi = static_cast<double>(deg);
            double testCost = 0.0;
            for (int i = 0; i < N; ++i) {
                const double predicted = A * patternLinear(headings[i] - testPhi) + B;
                const double r = snrValues[i] - predicted;
                testCost += r * r;
            }
            if (testCost < bestCost) {
                bestCost = testCost;
                phi = testPhi;
            }
        }
    }

    // Levenberg-Marquardt refinement of [phi, A, B]
    double lambda = 1.0;

    // Compute initial cost
    double cost = 0.0;
    for (int i = 0; i < N; ++i) {
        const double predicted = A * patternLinear(headings[i] - phi) + B;
        const double r = snrValues[i] - predicted;
        cost += r * r;
    }

    for (int iter = 0; iter < kMaxIterations; ++iter) {
        double JtJ[3][3] = {};
        double Jtr[3]    = {};

        for (int i = 0; i < N; ++i) {
            const double offset    = headings[i] - phi;
            const double pVal      = patternLinear(offset);
            const double predicted = A * pVal + B;
            const double residual  = snrValues[i] - predicted;

            // Numerical derivative of pattern w.r.t. phi (central difference)
            const double pPlus  = patternLinear(offset + kNumDiffStep);
            const double pMinus = patternLinear(offset - kNumDiffStep);
            // d/dphi: shifting phi up means offset decreases
            const double dPhi = -A * (pMinus - pPlus) / (2.0 * kNumDiffStep);
            const double dA   = pVal;
            const double dB   = 1.0;

            const double J[3] = { dPhi, dA, dB };

            for (int r = 0; r < 3; ++r) {
                Jtr[r] += J[r] * residual;
                for (int c = 0; c < 3; ++c) {
                    JtJ[r][c] += J[r] * J[c];
                }
            }
        }

        // Marquardt damping (multiplicative) — scales with curvature so that
        // parameters with different magnitudes (phi°, A, B) are damped proportionally.
        for (int k = 0; k < 3; ++k) {
            JtJ[k][k] *= (1.0 + lambda);
        }

        // Solve 3x3 system JtJ * delta = Jtr using Cramer's rule
        const double det =
            JtJ[0][0] * (JtJ[1][1] * JtJ[2][2] - JtJ[1][2] * JtJ[2][1]) -
            JtJ[0][1] * (JtJ[1][0] * JtJ[2][2] - JtJ[1][2] * JtJ[2][0]) +
            JtJ[0][2] * (JtJ[1][0] * JtJ[2][1] - JtJ[1][1] * JtJ[2][0]);

        if (std::abs(det) < 1e-15) break;

        const double invDet = 1.0 / det;

        double delta[3];
        delta[0] = invDet * (
            Jtr[0] * (JtJ[1][1] * JtJ[2][2] - JtJ[1][2] * JtJ[2][1]) -
            JtJ[0][1] * (Jtr[1] * JtJ[2][2] - JtJ[1][2] * Jtr[2]) +
            JtJ[0][2] * (Jtr[1] * JtJ[2][1] - JtJ[1][1] * Jtr[2]));
        delta[1] = invDet * (
            JtJ[0][0] * (Jtr[1] * JtJ[2][2] - JtJ[1][2] * Jtr[2]) -
            Jtr[0] * (JtJ[1][0] * JtJ[2][2] - JtJ[1][2] * JtJ[2][0]) +
            JtJ[0][2] * (JtJ[1][0] * Jtr[2] - Jtr[1] * JtJ[2][0]));
        delta[2] = invDet * (
            JtJ[0][0] * (JtJ[1][1] * Jtr[2] - Jtr[1] * JtJ[2][1]) -
            JtJ[0][1] * (JtJ[1][0] * Jtr[2] - Jtr[1] * JtJ[2][0]) +
            Jtr[0] * (JtJ[1][0] * JtJ[2][1] - JtJ[1][1] * JtJ[2][0]));

        // Clamp phi step to prevent overshooting past sharp nulls
        if (std::abs(delta[0]) > kMaxPhiStep) {
            const double scale = kMaxPhiStep / std::abs(delta[0]);
            delta[0] *= scale;
            delta[1] *= scale;
            delta[2] *= scale;
        }

        // Trial update
        const double phiTrial = phi + delta[0];
        const double ATrial   = std::max(A + delta[1], 0.0);
        const double BTrial   = B + delta[2];

        // Compute trial cost
        double trialCost = 0.0;
        for (int i = 0; i < N; ++i) {
            const double predicted = ATrial * patternLinear(headings[i] - phiTrial) + BTrial;
            const double r = snrValues[i] - predicted;
            trialCost += r * r;
        }

        if (trialCost < cost) {
            phi  = phiTrial;
            A    = ATrial;
            B    = BTrial;
            cost = trialCost;
            lambda *= 0.5;
            if (lambda < 1e-7) lambda = 1e-7;
        } else {
            lambda *= 4.0;
            if (lambda > 1e7) break;
            continue;
        }

        const double stepSize = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (stepSize < kConvergenceEps) break;
    }

    // Normalize phi to 0-360
    phi = std::fmod(phi, 360.0);
    if (phi < 0.0) phi += 360.0;

    // Compute R²
    double mean = 0.0;
    for (int i = 0; i < N; ++i) {
        mean += snrValues[i];
    }
    mean /= N;

    double ssRes = 0.0;
    double ssTot = 0.0;
    for (int i = 0; i < N; ++i) {
        const double predicted = A * patternLinear(headings[i] - phi) + B;
        const double residual  = snrValues[i] - predicted;
        ssRes += residual * residual;
        ssTot += (snrValues[i] - mean) * (snrValues[i] - mean);
    }

    const double rSquared = (ssTot > 1e-12) ? (1.0 - ssRes / ssTot) : 0.0;

    result.bearing_deg = static_cast<float>(phi);
    result.r_squared = static_cast<float>(std::max(0.0, rSquared));
    return result;
}
