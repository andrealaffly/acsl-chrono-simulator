###############################################################################
# Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
###############################################################################
#
###############################################################################
# File:        metrics.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Scores one simulator run from its controller log. Only the outer loop
#     is scored, with three objectives (all to be minimized):
#
#       position - RMS over time of ||position error||          [m]
#       velocity - RMS over time of ||velocity error||          [m/s]
#       effort   - RMS of every motor's saturated normalized
#                  thrust, over all motors and samples           [0-1]
#
#     It also records the fraction of thrust samples sitting at saturation
#     (reported in history.csv, not optimized).
#
#     CRASHES
#     The simulator always runs to the end of the trajectory, even after the
#     drone has gone unstable, so a crash has to be detected from the log:
#     the first NaN/inf in a scored column, or ||position error|| staying
#     above a threshold for a hold time -- whichever happens first.
#
#     SCORING (class Scorer)
#     A completed run's objectives are divided by the baseline dry run's
#     (so 1.0 means "as good as the baseline", 0.5 "twice as good") and
#     capped at max_normalized. A crashed run instead scores the survival
#     penalty used by the pychrono GA tuner on every objective:
#
#         penalty_base * (1 + (T - t_survival) / T)
#
#     where T is the trajectory length. That lies between penalty_base and
#     2 * penalty_base, so surviving longer is better, but any completed run
#     (at most max_normalized) beats any crashed one.
###############################################################################

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Optional

import numpy as np

from .log_reader import ColumnSpec, ControllerLog, find_thrust_columns

# The objectives, in the order they appear in every objective vector.
OBJECTIVES = ("position", "velocity", "effort")


@dataclass
class MetricSettings:
    """The ga-config.yaml settings that affect how a single log is measured."""

    ignore_initial_seconds: float   # samples before this time aren't scored (take-off transient)
    saturation_low: float           # thrust at or below this counts as saturated
    saturation_high: float          # thrust at or above this counts as saturated
    crash_position_error: float     # ||position error|| above this ...
    crash_hold_seconds: float       # ... for this long is a crash

    @classmethod
    def from_config(cls, config: dict) -> "MetricSettings":
        objectives = config.get("objectives") or {}
        crash = config.get("crash") or {}
        return cls(
            ignore_initial_seconds=float(objectives.get("ignore_initial_seconds", 0.0)),
            saturation_low=float(objectives.get("saturation_low", 0.01)),
            saturation_high=float(objectives.get("saturation_high", 0.99)),
            crash_position_error=float(crash.get("position_error_threshold", 2.0)),
            crash_hold_seconds=float(crash.get("hold_seconds", 0.5)),
        )


@dataclass
class RunMetrics:
    """What one run measured, in physical units (before normalization)."""

    position: float              # RMS ||position error|| [m]
    velocity: float              # RMS ||velocity error|| [m/s]
    effort: float                # RMS normalized thrust [-]
    saturation_fraction: float   # fraction of thrust samples at saturation
    final_time: float            # last valid controller time in the log [s]
    crashed: bool
    crash_time: Optional[float]  # last valid time up to the crash [s], if crashed
    crash_reason: str
    motors: int
    samples: int                 # samples in the scored window

    def raw(self) -> np.ndarray:
        """The three objectives in OBJECTIVES order, unnormalized."""
        return np.array([self.position, self.velocity, self.effort], dtype=float)

    def to_dict(self) -> dict:
        return asdict(self)


# =============================================================================
# Measuring one log
# =============================================================================
def _rms_norm(values: np.ndarray) -> float:
    """RMS over time of the per-sample Euclidean norm of a (samples, 3) array."""
    return float(np.sqrt(np.mean(np.sum(values ** 2, axis=1)))) if len(values) else float("nan")


def _find_crash(time: np.ndarray, position_norm: np.ndarray, finite: np.ndarray, settings: MetricSettings) -> tuple[Optional[int], str]:
    """Index of the first crashed sample (or None) and the reason.

    Two kinds of crash, whichever comes first:
      - the first sample with a non-finite value in a scored column;
      - the START of the first stretch where ||position error|| stays above
        the threshold for at least the hold time.
    """
    bad = np.flatnonzero(~finite)
    first_bad = int(bad[0]) if bad.size else None

    # Walk the samples, tracking where the current "above threshold" stretch
    # started; the first stretch that lasts hold_seconds is the crash.
    first_hold = start = None
    for index, above in enumerate(position_norm > settings.crash_position_error):
        if not above:
            start = None
            continue
        start = index if start is None else start
        if time[index] - time[start] >= settings.crash_hold_seconds:
            first_hold = start
            break

    if first_bad is not None and (first_hold is None or first_bad <= first_hold):
        return first_bad, "non-finite value in log"
    if first_hold is not None:
        return first_hold, f"position error above {settings.crash_position_error:g} m for {settings.crash_hold_seconds:g} s"
    return None, ""


def compute_metrics(log: ControllerLog, spec: ColumnSpec, settings: MetricSettings) -> RunMetrics:
    """Measure one run. The log must already have passed check_columns."""
    thrust_columns, _ = find_thrust_columns(log.columns, spec.thrust_pattern)
    time = log.column(spec.time)
    position_error = log.columns_array(spec.position_error)
    velocity_error = log.columns_array(spec.velocity_error)
    thrust = log.columns_array(thrust_columns)

    # A sample is "finite" if every scored value in it is a real number.
    finite = np.all(np.isfinite(np.hstack([position_error, velocity_error, thrust, time[:, None]])), axis=1)
    # NaN position errors count as infinitely large, so they also trip the
    # position-error threshold.
    position_norm = np.linalg.norm(np.nan_to_num(position_error, nan=np.inf), axis=1)
    crash_index, crash_reason = _find_crash(time, position_norm, finite, settings)

    finite_time = time[np.isfinite(time)]
    crash_time = None
    if crash_index is not None:
        # The last valid time up to and including the crash sample (a NaN
        # sample may have no valid time of its own).
        until_crash = time[: crash_index + 1]
        until_crash = until_crash[np.isfinite(until_crash)]
        crash_time = float(until_crash.max()) if until_crash.size else 0.0

    # The scored window: finite samples, after the ignored start-up transient
    # and before the crash (if any).
    window = finite & (time >= settings.ignore_initial_seconds)
    if crash_index is not None:
        window[crash_index:] = False
    scored_thrust = thrust[window]
    saturated = (scored_thrust <= settings.saturation_low) | (scored_thrust >= settings.saturation_high)

    return RunMetrics(
        position=_rms_norm(position_error[window]),
        velocity=_rms_norm(velocity_error[window]),
        effort=float(np.sqrt(np.mean(scored_thrust ** 2))) if scored_thrust.size else float("nan"),
        saturation_fraction=float(np.mean(saturated)) if scored_thrust.size else float("nan"),
        final_time=float(finite_time.max()) if finite_time.size else 0.0,
        crashed=crash_index is not None,
        crash_time=crash_time,
        crash_reason=crash_reason,
        motors=len(thrust_columns),
        samples=int(np.count_nonzero(window)),
    )


# =============================================================================
# Turning measurements into scores
# =============================================================================
class Scorer:
    """Maps RunMetrics to the optimizer's objective vector (normalized by the
    baseline dry run), and to the weighted sum used by the single-objective
    algorithms and for picking the single best solution."""

    def __init__(self, reference: RunMetrics, config: dict) -> None:
        objectives = config.get("objectives") or {}
        crash = config.get("crash") or {}
        weights = objectives.get("weights") or {}
        self.weights = np.array([float(weights.get(name, 0.0)) for name in OBJECTIVES])
        if not np.any(self.weights > 0):
            raise ValueError("ga-config.yaml: at least one objective weight must be positive")
        self.max_normalized = float(objectives.get("max_normalized", 100.0))
        self.penalty_base = float(crash.get("penalty_base", 1000.0))
        self.time_tolerance = float(crash.get("time_tolerance_seconds", 0.5))

        # Divisors for normalization. The tiny floor avoids dividing by zero
        # for an objective the baseline happens to score exactly 0 on.
        self.reference = np.maximum(np.abs(reference.raw()), 1e-9)
        # The baseline flew the whole trajectory, so its last logged time is
        # how long every candidate is expected to survive (T above).
        self.expected_duration = max(reference.final_time, 1e-9)
        self.baseline_score = self.weighted(self.objectives(reference))

    def ended_early(self, metrics: RunMetrics) -> bool:
        """True if the log stops well before the trajectory ends (killed run,
        simulator error). Such a run is scored like a crash at its last sample."""
        return metrics.final_time < self.expected_duration - self.time_tolerance

    def penalty(self, survival_time: float) -> np.ndarray:
        """The crash penalty on every objective (see the file header)."""
        deficit = float(np.clip((self.expected_duration - survival_time) / self.expected_duration, 0.0, 1.0))
        return np.full(len(OBJECTIVES), self.penalty_base * (1.0 + deficit))

    def objectives(self, metrics: Optional[RunMetrics]) -> np.ndarray:
        """The objective vector of a run. `metrics` is None when the run
        produced no usable log at all, which scores the worst penalty."""
        if metrics is None:
            return self.penalty(0.0)
        if metrics.crashed:
            return self.penalty(metrics.crash_time or 0.0)
        if self.ended_early(metrics):
            return self.penalty(metrics.final_time)
        raw = metrics.raw()
        # An objective that came out NaN (e.g. an empty scored window) is
        # treated as the worst normal score rather than poisoning the sum.
        raw = np.where(np.isfinite(raw), raw, self.max_normalized * self.reference)
        return np.clip(raw / self.reference, 0.0, self.max_normalized)

    def weighted(self, objectives: np.ndarray) -> float:
        """The weighted sum of an objective vector (lower is better)."""
        return float(np.dot(self.weights, objectives))
