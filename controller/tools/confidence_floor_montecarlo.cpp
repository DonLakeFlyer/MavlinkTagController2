// Monte-Carlo of BearingCalculator's composite confidence for a true lock,
// a noise-spike false lock and a flat interferer, per antenna, at K=20
// measurement noise. Prints percentiles and a suggested confidence floor so
// AntennaPattern::confidenceFloor can be tuned rather than guessed.
//
//   confidence_floor_montecarlo [--antenna ra2a|ra23k|all] [--k 20]
//                               [--trials 4000] [--seed 1]
//
// Model (per heading, unit noise power N=1, complex Gaussian noise):
//   true lock:   K pulses, each split over the 2-window footprint with
//                deterministic amplitude sqrt(S*g(theta-phi)/2) per window;
//                reported power = (sum|x|^2 - 2K) / K.
//   false lock:  no signal. The lock heading's value is the max over
//                M = 265 offsets x 116 bins of a K-window noise sum plus K
//                independent neighbour windows (what re-measuring the fold
//                winner at its own coordinates returns); other headings are
//                zero-mean noise sums. A K-window noise sum is drawn directly
//                as Gamma(K, N) rather than as K |CN(0,N)|^2 terms.
//   interferer:  pattern-free constant power S on every heading plus noise.

#include "AntennaPattern.h"
#include "BearingCalculator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr int    kHeadings      = 8;
constexpr int    kSearchOffsets = 265;   // one 2 s PRI in 7.55 ms STFT steps
// The acquisition gate is +/-2 kHz but Wf only spans +/-fs/2 = +/-1920 Hz at
// half-bin spacing (2 * n_w = 2 * 58 bins for a 15 ms pulse at 3840 S/s), so a
// centred tag searches every bin.
constexpr int    kStftWindow    = 58;
constexpr int    kSearchBins    = 2 * kStftWindow;
constexpr double kNoise         = 1.0;

struct Rng {
    std::mt19937_64 gen;
    std::normal_distribution<double> gauss { 0.0, 1.0 };
    explicit Rng(uint64_t seed) : gen(seed) {}
    // |a + n|^2 with n ~ CN(0, N): the power of one STFT window.
    double windowPower(double signalAmplitude) {
        const double re = signalAmplitude + gauss(gen) * std::sqrt(kNoise / 2.0);
        const double im = gauss(gen) * std::sqrt(kNoise / 2.0);
        return re * re + im * im;
    }
    double uniform(double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(gen);
    }
    int uniformInt(int lo, int hi) {
        return std::uniform_int_distribution<int>(lo, hi)(gen);
    }
    // Sum of k independent |CN(0,N)|^2 windows: each is Exp(N), so the sum is Gamma(k, N).
    double noiseWindowSum(int k) {
        return std::gamma_distribution<double>(static_cast<double>(k), kNoise)(gen);
    }
};

// Fixed-offset per-pulse power estimate over K pulses of per-pulse signal S.
double measuredPower(Rng& rng, int k, double signalPerPulse) {
    const double ampPerWindow = std::sqrt(std::max(0.0, signalPerPulse) / 2.0);
    double sum = 0.0;
    for (int i = 0; i < 2 * k; ++i) sum += rng.windowPower(ampPerWindow);
    return (sum - 2.0 * k * kNoise) / k;
}

// What re-measuring a noise fold-winner at its own coordinates returns.
double falseLockPeakPower(Rng& rng, int k) {
    double best = -1.0;
    for (int m = 0; m < kSearchOffsets * kSearchBins; ++m) {
        best = std::max(best, rng.noiseWindowSum(k));
    }
    const double neighbours = rng.noiseWindowSum(k);
    return (best + neighbours - 2.0 * k * kNoise) / k;
}

float confidenceOf(const AntennaPattern& pattern, const std::vector<double>& powers) {
    BearingCalculator calc(pattern);
    calc.setConfidenceFloor(0.0f);
    for (int h = 0; h < kHeadings; ++h) {
        calc.addSlice(static_cast<float>(h * 45), powers[h], 1, 0.0);
    }
    const auto results = calc.solve();
    return results.empty() ? 0.0f : results[0].r_squared;
}

double percentile(std::vector<float>& v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx = p / 100.0 * (v.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (v[hi] - v[lo]) * (idx - lo);
}

double fractionAtOrAbove(const std::vector<float>& v, double floor) {
    size_t n = 0;
    for (float x : v) if (x >= floor) ++n;
    return v.empty() ? 0.0 : static_cast<double>(n) / v.size();
}

void runAntenna(const AntennaPattern& pattern, int k, int trials, uint64_t seed) {
    Rng rng(seed);
    const double snrsDb[] = { 0.0, 3.0, 6.0, 10.0 };

    std::printf("=== %s  (K=%d, %d trials, floor now %.2f) ===\n",
                pattern.name, k, trials, pattern.confidenceFloor);
    std::printf("%-22s %7s %7s %7s   %s\n", "case", "p5", "p50", "p95", "kept @floor");

    std::vector<std::vector<float>> trueByLevel;
    for (double snrDb : snrsDb) {
        const double S = std::pow(10.0, snrDb / 10.0) * kNoise;
        std::vector<float> conf;
        conf.reserve(trials);
        for (int t = 0; t < trials; ++t) {
            const double phi = rng.uniform(0.0, 360.0);
            std::vector<double> powers(kHeadings);
            for (int h = 0; h < kHeadings; ++h) {
                const double g = BearingCalculator::patternLinear(pattern, h * 45.0 - phi);
                powers[h] = measuredPower(rng, k, S * g);
            }
            conf.push_back(confidenceOf(pattern, powers));
        }
        std::vector<float> sorted = conf;
        std::printf("%-22s %7.3f %7.3f %7.3f   %5.1f%%\n",
                    ("true lock " + std::to_string(static_cast<int>(snrDb)) + " dB").c_str(),
                    percentile(sorted, 5), percentile(sorted, 50), percentile(sorted, 95),
                    100.0 * fractionAtOrAbove(conf, pattern.confidenceFloor));
        trueByLevel.push_back(std::move(conf));
    }

    std::vector<float> falseConf;
    falseConf.reserve(trials);
    for (int t = 0; t < trials; ++t) {
        std::vector<double> powers(kHeadings);
        const int lockHeading = rng.uniformInt(0, kHeadings - 1);
        for (int h = 0; h < kHeadings; ++h) {
            powers[h] = h == lockHeading ? falseLockPeakPower(rng, k) : measuredPower(rng, k, 0.0);
        }
        falseConf.push_back(confidenceOf(pattern, powers));
    }
    {
        std::vector<float> sorted = falseConf;
        std::printf("%-22s %7.3f %7.3f %7.3f   %5.1f%%  (accepted)\n", "false lock (noise)",
                    percentile(sorted, 5), percentile(sorted, 50), percentile(sorted, 95),
                    100.0 * fractionAtOrAbove(falseConf, pattern.confidenceFloor));
    }

    std::vector<float> flatConf;
    flatConf.reserve(trials);
    for (double snrDb : { 3.0, 10.0 }) {
        const double S = std::pow(10.0, snrDb / 10.0) * kNoise;
        std::vector<float> conf;
        conf.reserve(trials);
        for (int t = 0; t < trials; ++t) {
            std::vector<double> powers(kHeadings);
            for (int h = 0; h < kHeadings; ++h) powers[h] = measuredPower(rng, k, S);
            conf.push_back(confidenceOf(pattern, powers));
        }
        std::vector<float> sorted = conf;
        std::printf("%-22s %7.3f %7.3f %7.3f   %5.1f%%  (accepted)\n",
                    ("flat interferer " + std::to_string(static_cast<int>(snrDb)) + " dB").c_str(),
                    percentile(sorted, 5), percentile(sorted, 50), percentile(sorted, 95),
                    100.0 * fractionAtOrAbove(conf, pattern.confidenceFloor));
        flatConf.insert(flatConf.end(), conf.begin(), conf.end());
    }

    // Floor that rejects 95% of false locks and flat interferers; report what
    // it costs in true locks at each level.
    std::vector<float> rejectSet = falseConf;
    rejectSet.insert(rejectSet.end(), flatConf.begin(), flatConf.end());
    const double suggested = percentile(rejectSet, 95);
    std::printf("suggested floor (rejects 95%% of false/flat): %.3f\n", suggested);
    for (size_t i = 0; i < trueByLevel.size(); ++i) {
        std::printf("  true lock %2d dB kept at suggested floor: %5.1f%%\n",
                    static_cast<int>(snrsDb[i]), 100.0 * fractionAtOrAbove(trueByLevel[i], suggested));
    }
    std::printf("\n");
}

}

int main(int argc, char** argv) {
    std::string antenna = "all";
    int k = 20;
    int trials = 4000;
    uint64_t seed = 1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--antenna") == 0 && i + 1 < argc) antenna = argv[++i];
        else if (std::strcmp(argv[i], "--k") == 0 && i + 1 < argc) k = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--trials") == 0 && i + 1 < argc) trials = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) seed = std::strtoull(argv[++i], nullptr, 10);
        else {
            std::fprintf(stderr, "usage: %s [--antenna ra2a|ra23k|all] [--k N] [--trials N] [--seed N]\n", argv[0]);
            return 2;
        }
    }
    if (k < 2 || trials < 1) {
        std::fprintf(stderr, "--k must be >= 2 and --trials >= 1\n");
        return 2;
    }
    if (antenna == "ra2a" || antenna == "all") runAntenna(AntennaPatterns::ra2a(), k, trials, seed);
    if (antenna == "ra23k" || antenna == "all") runAntenna(AntennaPatterns::ra23k(), k, trials, seed);
    if (antenna != "ra2a" && antenna != "ra23k" && antenna != "all") {
        std::fprintf(stderr, "unknown antenna '%s'\n", antenna.c_str());
        return 2;
    }
    return 0;
}
