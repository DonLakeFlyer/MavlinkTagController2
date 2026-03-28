# Rate Switch Aware Detector Design

## Overview

This document describes detector implementations that can handle pulse repetition rate changes during an active detection window, instead of assuming a single fixed inter-pulse interval per cycle.

The current detector flow is documented in `README.md`, and cross-rate leakage behavior is described in `CROSS_RATE_REJECTION.md`. This design extends that flow with explicit rate-switch modeling.

## Problem Statement

A collar can switch between two pulse repetition intervals during motion or behavior change. If a detector assumes a single PRI for the full segment, a mid-segment switch causes partial fold misalignment and temporary sensitivity loss, especially for weak long-range signals.

Goal:
- Detect both stable-rate and switch-rate segments with minimal long-range sensitivity loss.

Non-goals:
- Full blind search over arbitrary unknown pulse schedules.
- Large compute growth from dense brute-force rate banks.

## Recommended Implementation

Use a single detector process with one STFT pass and a multi-hypothesis fold bank that includes change-point schedules.

Why:
- Reuses existing STFT and noise/threshold pipeline in `pulse_detector.py`.
- Models the exact event of interest: one rate switching to another within a segment.
- Lower integration risk than full adaptive trackers.

## Signal Model

Let:
- $P(f,t)$ be STFT power at frequency bin $f$ and time-window index $t$.
- $K$ be fold count (currently 5).
- $N_A$ and $N_B$ be window spacings for the two PRIs.
- $t_0$ be first-pulse offset candidate.

A hypothesis defines an inter-pulse schedule $\sigma = [\sigma_0,\sigma_1,\dots,\sigma_{K-2}]$ of length $K-1$, where each $\sigma_k \in \{A,B\}$.

Pulse window indices:
$$
i_0 = t_0,\quad
i_k = t_0 + \sum_{j=0}^{k-1} N_{\sigma_j},\; k \ge 1
$$

Fold score:
$$
S_h(f,t_0) = \sum_{k=0}^{K-1} P(f, i_k)
$$

For two rates and one switch max, hypothesis set is:
- Pure A
- Pure B
- A to B at change-point $c \in \{1,\dots,K-2\}$ (non-degenerate switches only)
- B to A at change-point $c \in \{1,\dots,K-2\}$ (non-degenerate switches only)

For $K=5$, the total number of hypotheses is 8.

## Detection Statistic and Thresholding

For each frequency bin:
- Evaluate all valid $t_0$ and hypotheses.
- Keep best hypothesis score:
$$
S^\star(f) = \max_{h,t_0} S_h(f,t_0)
$$

Decision:
- Compare $S^\star(f)$ to per-bin threshold derived from EVT and per-bin noise scaling, consistent with current method in `README.md`.

Important:
- EVT calibration must include the larger search space (all hypotheses and offsets), otherwise false alarm rate will be underestimated.

## Uniformity and Robustness Filters

Apply post-threshold checks to reject cross-rate leakage and pathological patterns.

On-window uniformity:
$$
U_h = \frac{\min_k P(f,i_k)}{\max_k P(f,i_k)}
$$
Reject if $U_h < U_{min}$.

Recommended:
- Start with $U_{min}=0.25$ (same rationale as `CROSS_RATE_REJECTION.md`).
- Expose as runtime parameter for tuning in harsh multipath environments.

Optional switch penalty:
$$
J_h = S_h - \lambda \cdot n_{switch}(h)
$$
Use $J_h$ for ranking to avoid over-preferring switch hypotheses in noise. Keep penalty small and validate with replay.

## Long-Range Sensitivity Safeguards

To preserve weak-signal performance:
- Keep hypothesis family small (pure plus single-switch only).
- Prefer soft penalties over hard rejection.
- Require only one robust uniformity threshold, not multiple strict filters.
- Maintain current per-bin adaptive threshold scaling.
- Validate at low SNR with synthetic and replay data.

Expected behavior:
- Better than fixed-PRI detector during transition windows.
- Similar sensitivity to current detector in steady-rate segments.

## Complexity and Performance

Compute increase is mainly in fold stage, not STFT:
- STFT cost unchanged (single pass).
- Fold cost scales by number of hypotheses.
- For two-rate single-switch bank, fold work is about 10x current single-hypothesis fold, but still small relative to SDR and transport pipeline in typical settings.

## Interface and Configuration

Add parameters:
- `--tip-secondary`: secondary inter-pulse interval (seconds).
- `--enable-rate-switch-bank`: enable change-point hypotheses.
- `--max-switches-per-segment`: default 1. Values greater than 1 are not supported in the initial single-switch implementation and are reserved for future multi-switch schedules.
- `--min-uniformity`: default 0.25.
- `--switch-penalty`: default small positive value.

Backwards compatibility:
- If `--tip-secondary` is absent, behavior matches current single-rate detector.

## Integration Plan

1. Add hypothesis generation and index construction in fold stage in `pulse_detector.py`.
2. Update EVT threshold generation to calibrate over full hypothesis bank.
3. Add uniformity check at the integration point described in `CROSS_RATE_REJECTION.md`.
4. Extend logs with selected hypothesis type:
   - A
   - B
   - A_to_B_cX
   - B_to_A_cX
5. Keep UDP reporting format unchanged initially. Include hypothesis label in logs first.

## Validation Plan

Unit tests:
- Index generation for pure and switch hypotheses.
- Boundary conditions for valid offsets and segment edges.
- Uniformity metric correctness.

Monte Carlo:
- Verify empirical false alarm rate matches target pf after hypothesis-bank EVT calibration.

Scenario tests:
- Stable A only.
- Stable B only.
- A to B switch mid-segment.
- B to A switch mid-segment.
- Cross-rate strong interferer.
- Low-SNR long-range pulses with and without fading.

Acceptance criteria:
- No steady-state sensitivity regression versus current detector.
- Reduced missed detections across rate-switch transitions.
- Controlled false alarm behavior at configured pf.

## Alternative Implementations

Adaptive PRI tracker:
- Estimates PRI from detected pulse times each cycle.
- Better for unknown rates but less stable at low SNR.

Dense multi-rate bank:
- Covers many PRIs and switch windows.
- High compute and stricter multiple-testing correction burden.

HMM or IMM state model:
- Strong theoretical framework for switching dynamics.
- Higher implementation and tuning complexity.

## Recommendation

Implement the single-process multi-hypothesis fold bank with one-switch schedules first. It directly targets mid-window rate changes, minimizes architecture disruption, and is most likely to preserve long-range detection performance.
