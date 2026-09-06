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

void BearingCalculator::addSlice(float heading_deg, double signal_power, uint32_t tag_id,
                                 double snr_db)
{
    _slices.push_back({heading_deg, signal_power, tag_id, snr_db, true});
}

void BearingCalculator::addNoDetection(float heading_deg, uint32_t tag_id)
{
    _slices.push_back({heading_deg, 0.0, tag_id, 0.0, false});
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

// Least-squares fit of power(θ) = A·pattern(θ-φ) + B for a fixed φ.
// B is only free when there are enough detections to constrain it; otherwise
// it is pinned at 0 (signal_power is already noise-subtracted).
void BearingCalculator::_fitAmplitude(const std::vector<double>& g,
                                      const std::vector<double>& p,
                                      bool fitFloor, double& A, double& B)
{
    const int n = static_cast<int>(p.size());
    double sg = 0.0, sgg = 0.0, sp = 0.0, sgp = 0.0;
    for (int i = 0; i < n; ++i) {
        sg  += g[i];
        sgg += g[i] * g[i];
        sp  += p[i];
        sgp += g[i] * p[i];
    }
    if (fitFloor) {
        const double det = n * sgg - sg * sg;
        if (std::abs(det) > 1e-12) {
            A = (n * sgp - sg * sp) / det;
            B = (sp - A * sg) / n;
            if (A >= 0.0 && B >= 0.0) return;
            if (A >= 0.0) {
                // Negative floor is unphysical and would let the model predict
                // zero power at censored headings; fall back to the B=0 boundary.
                B = 0.0;
                A = sgg > 1e-12 ? std::max(0.0, sgp / sgg) : 0.0;
                return;
            }
        }
        A = 0.0;
        B = std::max(0.0, sp / n);
        return;
    }
    B = 0.0;
    A = sgg > 1e-12 ? std::max(0.0, sgp / sgg) : 0.0;
}

BearingCalculator::Result BearingCalculator::_solveForTag(uint32_t tag_id, const std::vector<SliceData>& slices) const
{
    Result result {};
    result.tag_id = tag_id;
    result.bearing_deg = std::numeric_limits<float>::quiet_NaN();
    result.r_squared = 0.0f;

    std::vector<double> detHeadings, detPowers, censHeadings;
    double bestSnr = -1e9;
    double bestPower = -std::numeric_limits<double>::infinity();
    double minPositivePower = std::numeric_limits<double>::infinity();
    float bestHeading = 0;
    for (const auto& s : slices) {
        if (!s.detected) {
            censHeadings.push_back(s.heading_deg);
            continue;
        }
        detHeadings.push_back(s.heading_deg);
        detPowers.push_back(s.signal_power);
        bestSnr = std::max(bestSnr, s.snr_db);
        if (s.signal_power > bestPower) {
            bestPower = s.signal_power;
            bestHeading = s.heading_deg;
        }
        if (s.signal_power > 0.0) {
            minPositivePower = std::min(minPositivePower, s.signal_power);
        }
    }

    const int nDet  = static_cast<int>(detHeadings.size());
    const int nCens = static_cast<int>(censHeadings.size());
    result.n_valid_slices = static_cast<uint32_t>(nDet);
    result.best_snr = static_cast<float>(nDet > 0 ? bestSnr : 0.0);

    if (nDet == 0 || bestPower <= 0.0) {
        // Nothing usable; report the strongest heading if there was one.
        if (nDet > 0) result.bearing_deg = bestHeading;
        return result;
    }

    // Undetected headings had power below (roughly) the weakest detection.
    const double censorLimit = kCensorFraction * minPositivePower;
    const bool fitFloor = nDet >= kMinDetectedForFloor;

    double sumPowerSq = 0.0;
    for (double p : detPowers) sumPowerSq += p * p;

    // The strongest detection must lie in the front half of the pattern, so
    // only scan φ within ±90° of it. This also breaks the front/back mirror
    // ambiguity of the symmetric pattern.
    const int nSteps = static_cast<int>(std::lround(180.0 / kScanStepDeg)) + 1;
    std::vector<double> costs(nSteps);
    std::vector<double> g(nDet);
    double A = 0.0, B = 0.0;
    int bestIdx = 0;
    double minCost = std::numeric_limits<double>::max();
    for (int k = 0; k < nSteps; ++k) {
        const double phi = bestHeading - 90.0 + k * kScanStepDeg;
        for (int i = 0; i < nDet; ++i) g[i] = patternLinear(detHeadings[i] - phi);
        _fitAmplitude(g, detPowers, fitFloor, A, B);

        double cost = 0.0;
        for (int i = 0; i < nDet; ++i) {
            const double r = detPowers[i] - (A * g[i] + B);
            cost += r * r;
        }
        for (int j = 0; j < nCens; ++j) {
            const double excess = A * patternLinear(censHeadings[j] - phi) + B - censorLimit;
            if (excess > 0.0) cost += excess * excess;
        }
        costs[k] = cost;
        if (cost < minCost) {
            minCost = cost;
            bestIdx = k;
        }
    }

    // Plausible set: every φ whose cost is within a factor of two of the
    // minimum (with a small floor so exact fits still get a nonzero width).
    const double threshold = minCost + std::max(minCost, kSpanFloorFraction * sumPowerSq);
    int lo = bestIdx, hi = bestIdx;
    while (lo > 0 && costs[lo - 1] <= threshold) --lo;
    while (hi < nSteps - 1 && costs[hi + 1] <= threshold) ++hi;
    const double spanDeg = (hi - lo) * kScanStepDeg;

    double phi = bestHeading - 90.0 + 0.5 * (lo + hi) * kScanStepDeg;
    phi = std::fmod(phi, 360.0);
    if (phi < 0.0) phi += 360.0;

    // Confidence: fraction of detected energy explained, scaled by how
    // tightly the data pin φ and by how many observations exceed the model's
    // parameter count. A lone detection with nothing else scores 0; two
    // detections fit exactly but are still sparse.
    const int nParams = fitFloor ? 3 : 2;
    const double fitFactor  = std::clamp(1.0 - minCost / sumPowerSq, 0.0, 1.0);
    const double spanFactor = std::clamp(1.0 - spanDeg / 180.0, 0.0, 1.0);
    const double dofFactor  = std::clamp((nDet + nCens - nParams + 1) / kFullDof, 0.0, 1.0);

    result.bearing_deg = static_cast<float>(phi);
    result.r_squared = static_cast<float>(fitFactor * spanFactor * dofFactor);
    return result;
}
