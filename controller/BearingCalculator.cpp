#include "BearingCalculator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

BearingCalculator::BearingCalculator(const AntennaPattern& pattern)
    : _pattern(pattern)
    , _confidenceFloor(pattern.confidenceFloor)
{
}

std::optional<float> BearingCalculator::revisitHeadingFor(const std::vector<Result>& results)
{
    for (const auto& result : results) {
        if (!result.rejected && std::isfinite(result.bearing_deg)
            && result.n_sighted_slices == 1) {
            return result.bearing_deg;
        }
    }
    return std::nullopt;
}

uint32_t BearingCalculator::largestFrequencyCluster(const std::vector<double>& freqsHz, double toleranceHz)
{
    uint32_t best = 0;
    for (const double centre : freqsHz) {
        if (!std::isfinite(centre)) continue;
        uint32_t n = 0;
        for (const double f : freqsHz) {
            if (std::isfinite(f) && std::fabs(f - centre) <= toleranceHz) ++n;
        }
        best = std::max(best, n);
    }
    return best;
}

void BearingCalculator::addSlice(float heading_deg, double signal_power, uint32_t tag_id,
                                 double snr_db, uint8_t candidate_id, bool sighted,
                                 double noise_psd)
{
    _slices.push_back({heading_deg, signal_power, tag_id, snr_db, true, candidate_id, sighted, noise_psd});
}

void BearingCalculator::addNoDetection(float heading_deg, uint32_t tag_id, double noise_psd)
{
    _slices.push_back({heading_deg, 0.0, tag_id, 0.0, false, 0, false, noise_psd});
}

std::vector<double> BearingCalculator::sliceWeights(const std::vector<SliceData>& slices)
{
    std::vector<double> known;
    for (const auto& s : slices) {
        if (std::isfinite(s.noise_psd) && s.noise_psd > 0.0) known.push_back(s.noise_psd);
    }
    std::vector<double> weights(slices.size(), 1.0);
    if (known.empty()) return weights;
    std::sort(known.begin(), known.end());
    const double reference = known[known.size() / 2];
    for (size_t i = 0; i < slices.size(); ++i) {
        const double n = slices[i].noise_psd;
        if (std::isfinite(n) && n > 0.0) {
            const double ratio = reference / n;
            weights[i] = ratio * ratio;
        }
    }
    return weights;
}

void BearingCalculator::reset()
{
    _slices.clear();
}

// Interpolate the pattern table at an arbitrary angle offset from boresight (degrees).
// Returns the pattern value in linear (power) scale, normalized so boresight = 1.0.
double BearingCalculator::patternLinear(const AntennaPattern& pattern, double offsetDeg)
{
    // Normalize to 0-360 then fold to 0-180 (symmetric)
    double angle = std::fmod(offsetDeg, 360.0);
    if (angle < 0.0) angle += 360.0;
    if (angle > 180.0) angle = 360.0 - angle;

    // Interpolate in the 10° step table
    const double indexF = angle / 10.0;
    const int idx0 = static_cast<int>(indexF);
    const int idx1 = std::min(idx0 + 1, AntennaPattern::kSize - 1);
    const double frac = indexF - idx0;

    const double db = pattern.patternDb[idx0] * (1.0 - frac) + pattern.patternDb[idx1] * frac;
    return std::pow(10.0, db / 10.0);
}

std::vector<BearingCalculator::Result> BearingCalculator::solveCandidates() const
{
    // Detections group by (tag, candidate); no-detections are censored
    // observations shared by every candidate of their tag.
    std::map<uint32_t, std::vector<SliceData>> noDetections;
    std::map<std::pair<uint32_t, uint8_t>, std::vector<SliceData>> groups;
    for (const auto& s : _slices) {
        if (s.detected) {
            groups[{s.tag_id, s.candidate_id}].push_back(s);
        } else {
            noDetections[s.tag_id].push_back(s);
        }
    }
    // A tag with only no-detections still gets a (NaN) result.
    for (const auto& [tagId, slices] : noDetections) {
        groups.try_emplace({tagId, 0});
    }

    std::map<uint32_t, uint32_t> candidateCounts;
    for (const auto& [key, slices] : groups) {
        candidateCounts[key.first]++;
    }

    std::vector<Result> results;
    for (const auto& [key, slices] : groups) {
        std::vector<SliceData> fitSlices = slices;
        const auto censored = noDetections.find(key.first);
        if (censored != noDetections.end()) {
            fitSlices.insert(fitSlices.end(), censored->second.begin(), censored->second.end());
        }
        Result result = _solveForTag(key.first, fitSlices);
        result.candidate_id = key.second;
        result.n_candidates = candidateCounts[key.first];
        result.rejected = false;
        results.push_back(result);
    }
    return results;
}

bool BearingCalculator::_isBetterCandidate(const Result& candidate, const Result& incumbent)
{
    const float delta = candidate.r_squared - incumbent.r_squared;
    if (std::fabs(delta) > kCandidateConfidenceTolerance) {
        return delta > 0.0f;
    }
    if (candidate.n_sighted_slices != incumbent.n_sighted_slices) {
        return candidate.n_sighted_slices > incumbent.n_sighted_slices;
    }
    if (candidate.n_valid_slices != incumbent.n_valid_slices) {
        return candidate.n_valid_slices > incumbent.n_valid_slices;
    }
    return candidate.best_snr > incumbent.best_snr;
}

std::vector<BearingCalculator::Result> BearingCalculator::solve() const
{
    std::map<uint32_t, Result> best;
    for (const auto& candidate : solveCandidates()) {
        auto it = best.find(candidate.tag_id);
        if (it == best.end()) {
            best.emplace(candidate.tag_id, candidate);
        } else if (_isBetterCandidate(candidate, it->second)) {
            it->second = candidate;
        }
    }

    std::vector<Result> results;
    for (auto& [tagId, result] : best) {
        if (result.n_valid_slices > 0 && result.r_squared < _confidenceFloor) {
            result.rejected = true;
            result.bearing_deg = std::numeric_limits<float>::quiet_NaN();
        }
        results.push_back(result);
    }
    return results;
}

// Weighted least-squares fit of power(θ) = A·pattern(θ-φ) + B for a fixed φ.
// B is only free when there are enough detections to constrain it; otherwise
// it is pinned at 0 (signal_power is already noise-subtracted).
void BearingCalculator::_fitAmplitude(const std::vector<double>& g,
                                      const std::vector<double>& p,
                                      const std::vector<double>& w,
                                      bool fitFloor, double& A, double& B)
{
    const int n = static_cast<int>(p.size());
    double sw = 0.0, swg = 0.0, swgg = 0.0, swp = 0.0, swgp = 0.0;
    for (int i = 0; i < n; ++i) {
        sw   += w[i];
        swg  += w[i] * g[i];
        swgg += w[i] * g[i] * g[i];
        swp  += w[i] * p[i];
        swgp += w[i] * g[i] * p[i];
    }
    if (fitFloor) {
        const double det = sw * swgg - swg * swg;
        if (std::abs(det) > 1e-12 && sw > 0.0) {
            A = (sw * swgp - swg * swp) / det;
            B = (swp - A * swg) / sw;
            if (A >= 0.0 && B >= 0.0) return;
            if (A >= 0.0) {
                // Negative floor is unphysical and would let the model predict
                // zero power at censored headings; fall back to the B=0 boundary.
                B = 0.0;
                A = swgg > 1e-12 ? std::max(0.0, swgp / swgg) : 0.0;
                return;
            }
        }
        A = 0.0;
        B = sw > 0.0 ? std::max(0.0, swp / sw) : 0.0;
        return;
    }
    B = 0.0;
    A = swgg > 1e-12 ? std::max(0.0, swgp / swgg) : 0.0;
}

BearingCalculator::Result BearingCalculator::_solveForTag(uint32_t tag_id, const std::vector<SliceData>& slices) const
{
    Result result {};
    result.tag_id = tag_id;
    result.bearing_deg = std::numeric_limits<float>::quiet_NaN();
    result.r_squared = 0.0f;

    std::vector<double> detHeadings, detPowers, detWeights, censHeadings, censWeights;
    const std::vector<double> weights = sliceWeights(slices);
    double bestSnr = -1e9;
    double bestPower = -std::numeric_limits<double>::infinity();
    double minPositivePower = std::numeric_limits<double>::infinity();
    float bestHeading = 0;
    uint32_t nSighted = 0;
    for (size_t i = 0; i < slices.size(); ++i) {
        const auto& s = slices[i];
        if (!s.detected) {
            censHeadings.push_back(s.heading_deg);
            censWeights.push_back(weights[i]);
            continue;
        }
        detHeadings.push_back(s.heading_deg);
        detPowers.push_back(s.signal_power);
        detWeights.push_back(weights[i]);
        bestSnr = std::max(bestSnr, s.snr_db);
        if (s.sighted) ++nSighted;
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
    result.n_sighted_slices = nSighted;
    result.best_snr = static_cast<float>(nDet > 0 ? bestSnr : 0.0);

    if (nDet == 0 || bestPower <= 0.0) {
        // Nothing usable; report the strongest heading if there was one.
        if (nDet > 0) result.bearing_deg = bestHeading;
        return result;
    }

    // Undetected headings had power below (roughly) the weakest detection.
    const double censorLimit = kCensorFraction * minPositivePower;
    const bool fitFloor = nDet >= kMinDetectedForFloor;

    // Weighted energy: the reference the cost is judged against.
    double sumPowerSq = 0.0;
    for (int i = 0; i < nDet; ++i) sumPowerSq += detWeights[i] * detPowers[i] * detPowers[i];

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
        _fitAmplitude(g, detPowers, detWeights, fitFloor, A, B);

        double cost = 0.0;
        for (int i = 0; i < nDet; ++i) {
            const double r = detPowers[i] - (A * g[i] + B);
            cost += detWeights[i] * r * r;
        }
        for (int j = 0; j < nCens; ++j) {
            const double excess = A * patternLinear(censHeadings[j] - phi) + B - censorLimit;
            if (excess > 0.0) cost += censWeights[j] * excess * excess;
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

    // Residuals at the reported bearing, for the candidate log.
    for (int i = 0; i < nDet; ++i) g[i] = patternLinear(detHeadings[i] - phi);
    _fitAmplitude(g, detPowers, detWeights, fitFloor, A, B);
    result.residuals.reserve(nDet);
    for (int i = 0; i < nDet; ++i) {
        result.residuals.push_back({static_cast<float>(detHeadings[i]),
                                    detPowers[i] - (A * g[i] + B), detWeights[i]});
    }
    return result;
}
