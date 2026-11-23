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
# File:        log_reader.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Reads a controller's controller_log.log and checks it against the GA's
#     "column contract" (the "columns" section of config/ga-config.yaml).
#
#     Every line of the log is written by Boost.Log as
#         [timestamp] [thread] [ControllerTag] [process] [line] , v1, v2, ...,
#     The first line is the header (the column names) and every other line
#     is one sample.
#
#     Columns are looked up by their exact header name, never by position,
#     so a controller can log extra columns, or log them in any order, and
#     still be tuned. The motor thrust columns are found with a regular
#     expression ("Normalized Thrust 1 [-]", "Normalized Thrust 2 [-]", ...),
#     so any number of motors works.
###############################################################################

from __future__ import annotations

import re
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

CONTROLLER_LOG_NAME = "controller_log.log"

# The five "[...]" fields Boost.Log puts in front of every message...
_PREFIX_RE = re.compile(r"^(?:\[[^\]]*\]\s*){5}")
# ...the first of which is the timestamp.
_TIMESTAMP_RE = re.compile(r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\]")


class LogError(Exception):
    """The log is missing, empty, or has no usable samples."""


# =============================================================================
# The column contract
# =============================================================================
@dataclass
class ColumnSpec:
    """The column names the GA scores a run from (ga-config.yaml "columns")."""

    time: str                    # controller time [s]
    position_error: list[str]    # x, y, z position error [m]
    velocity_error: list[str]    # x, y, z velocity error [m/s]
    thrust_pattern: str          # regex with one group capturing the motor number

    @classmethod
    def from_config(cls, config: dict) -> "ColumnSpec":
        """Build the spec from the GA config, raising ValueError if it's incomplete."""
        columns = config.get("columns") or {}
        spec = cls(
            time=str(columns.get("time", "")),
            position_error=[str(name) for name in columns.get("position_error") or []],
            velocity_error=[str(name) for name in columns.get("velocity_error") or []],
            thrust_pattern=str(columns.get("thrust_pattern", "")),
        )
        if not (spec.time and spec.position_error and spec.velocity_error and spec.thrust_pattern):
            raise ValueError("ga-config.yaml: 'columns' needs time, position_error, velocity_error and thrust_pattern")
        try:
            groups = re.compile(spec.thrust_pattern).groups
        except re.error as error:
            raise ValueError(f"ga-config.yaml: invalid thrust_pattern: {error}") from error
        if groups < 1:
            raise ValueError("ga-config.yaml: thrust_pattern needs one capture group for the motor number")
        return spec

    @property
    def required(self) -> list[str]:
        """Every fixed column name the log must contain."""
        return [self.time, *self.position_error, *self.velocity_error]


@dataclass
class ColumnCheck:
    """Result of checking a log header against a ColumnSpec."""

    missing: list[str] = field(default_factory=list)          # required columns not in the header
    thrust_columns: list[str] = field(default_factory=list)   # found thrust columns, motor 1 first
    problems: list[str] = field(default_factory=list)         # anything else wrong (thrust numbering)

    @property
    def ok(self) -> bool:
        return not self.missing and not self.problems

    def describe(self) -> str:
        """One line for the console / GUI."""
        if self.ok:
            return f"GA-ready ({len(self.thrust_columns)} motor thrust columns)"
        missing = ["missing columns: " + ", ".join(f"'{name}'" for name in self.missing)] if self.missing else []
        return "; ".join(missing + self.problems)


def find_thrust_columns(columns: list[str], pattern: str) -> tuple[list[str], list[str]]:
    """Find the motor thrust columns in a header.

    Returns (the thrust columns ordered by motor number, problems). The
    motors must be numbered 1..N without gaps -- whatever N is -- or the
    numbering problem is reported.
    """
    regex = re.compile(pattern)
    numbered: dict[int, str] = {}
    for name in columns:
        match = regex.match(name)
        if match and match.group(1) and match.group(1).isdigit():
            numbered.setdefault(int(match.group(1)), name)   # the first column wins if a number repeats
    if not numbered:
        return [], [f"no thrust columns match '{pattern}'"]
    problems = []
    if sorted(numbered) != list(range(1, len(numbered) + 1)):
        problems.append(f"thrust columns must be numbered 1..N without gaps, found motors {sorted(numbered)}")
    return [numbered[number] for number in sorted(numbered)], problems


def check_columns(columns: list[str], spec: ColumnSpec) -> ColumnCheck:
    """Check a header (list of column names) against the contract."""
    thrust, problems = find_thrust_columns(columns, spec.thrust_pattern)
    return ColumnCheck(missing=[name for name in spec.required if name not in columns], thrust_columns=thrust, problems=problems)


# =============================================================================
# Reading the log
# =============================================================================
def _fields(line: str) -> Optional[list[str]]:
    """The comma-separated fields of one log line, without the Boost.Log
    prefix. None if the line has no prefix (not a log line).

    Every message starts with ", " and ends with ",", which leaves an empty
    field at each end after splitting; those two are dropped. Empty fields
    in the middle are kept, so columns stay aligned.
    """
    match = _PREFIX_RE.match(line)
    if not match:
        return None
    fields = [item.strip() for item in line[match.end():].strip().split(",")]
    return [item for index, item in enumerate(fields) if item or 0 < index < len(fields) - 1]


def read_header(path: Path) -> list[str]:
    """Just the column names (the first line) of a log."""
    try:
        with path.open("r", errors="replace") as handle:
            header = _fields(handle.readline())
    except OSError as error:
        raise LogError(f"Could not read {path}: {error}") from error
    if not header:
        raise LogError(f"{path} has no header line")
    return header


@dataclass
class ControllerLog:
    """A whole log as a numeric table."""

    path: Path
    columns: list[str]
    data: np.ndarray  # shape (samples, columns); unparsable values are NaN

    def column(self, name: str) -> np.ndarray:
        """One column, by header name."""
        return self.data[:, self.columns.index(name)]

    def columns_array(self, names: list[str]) -> np.ndarray:
        """Several columns, by header name, as a (samples, len(names)) array."""
        return self.data[:, [self.columns.index(name) for name in names]]


def _to_float(text: str) -> float:
    try:
        return float(text)
    except ValueError:
        return float("nan")


def read_controller_log(path: Path) -> ControllerLog:
    """Read a controller log into a ControllerLog."""
    if not path.is_file():
        raise LogError(f"Controller log not found: {path}")
    with path.open("r", errors="replace") as handle:
        lines = [fields for fields in map(_fields, handle) if fields is not None]
    if not lines:
        raise LogError(f"{path} is empty")
    header, rows = lines[0], lines[1:]
    if not rows:
        raise LogError(f"{path} has a header but no samples")

    # A run that was killed can leave a truncated last line, so only rows
    # with the most common field count are kept. And if a controller's data
    # rows and header disagree in length, only the columns both cover are
    # usable.
    width = Counter(len(row) for row in rows).most_common(1)[0][0]
    rows = [row for row in rows if len(row) == width]
    width = min(width, len(header))
    data = np.array([[_to_float(value) for value in row[:width]] for row in rows], dtype=float)
    return ControllerLog(path=path, columns=header[:width], data=data)


# =============================================================================
# Finding previous logs (for pre-flight and the GUI's runtime estimate)
# =============================================================================
def wall_clock_duration(path: Path) -> Optional[float]:
    """How long a run took in real time: the seconds between the first and
    last line's Boost.Log timestamps. None if the log has no timestamps."""
    first = last = None
    try:
        with path.open("r", errors="replace") as handle:
            for line in handle:
                match = _TIMESTAMP_RE.match(line)
                if match:
                    last = match.group(1)
                    first = first or last
    except OSError:
        return None
    if first is None or last is None:
        return None
    fmt = "%Y-%m-%d %H:%M:%S.%f"
    return (datetime.strptime(last, fmt) - datetime.strptime(first, fmt)).total_seconds()


def latest_controller_log(platform_log_dir: Path, controller_name: str) -> Optional[Path]:
    """The most recently written controller_log.log of this controller
    anywhere under sim-log/<platform>/ (normal and wrapper runs alike)."""
    if not platform_log_dir.is_dir():
        return None
    candidates = list(platform_log_dir.glob(f"**/{controller_name}/{CONTROLLER_LOG_NAME}"))
    return max(candidates, key=lambda candidate: candidate.stat().st_mtime) if candidates else None
