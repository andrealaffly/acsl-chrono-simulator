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
# File:        reporting.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Everything a GA session reports, in two forms:
#
#       - Console lines, in the same colored "[INFO] [--ACSL ...--]" style as
#         scripts/wrapper.py. Every line is flushed immediately: when the GUI
#         runs the GA, stdout is a pipe (block-buffered), and unflushed lines
#         would only show up in the GUI's console when the session ends.
#
#       - Files in the session folder sim-log/<platform>/ga/<timestamp>/:
#           session.yaml   the resolved config the session ran with
#           progress.json  the current state; the GUI's GA page polls it
#           history.csv    one row per evaluated candidate
#           pareto.json    the non-dominated candidates (their gains files
#                          are under pareto/), and which one is the best
#           console/       each simulator run's console output
###############################################################################

from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Any, Optional

from .gains_store import write_atomic

# ANSI colors (the same codes scripts/wrapper.py uses, so GA and wrapper
# output look alike in a terminal and in the GUI's console).
COLOR_BLUE = "\033[1;34m"
COLOR_GREEN = "\033[1;32m"
COLOR_RED = "\033[1;31m"
COLOR_ORANGE = "\033[38;5;214m"
COLOR_RESET = "\033[0m"

TAG = "[--ACSL GA--]"


# =============================================================================
# Console
# =============================================================================
def info(message: str) -> None:
    print(f"{COLOR_GREEN}[INFO] {TAG} {message}{COLOR_RESET}", flush=True)


def warning(message: str) -> None:
    print(f"{COLOR_ORANGE}[WARN] {TAG} {message}{COLOR_RESET}", flush=True)


def error(message: str) -> None:
    print(f"{COLOR_RED}[ERROR] {TAG} {message}{COLOR_RESET}", flush=True)


def status(message: str) -> None:
    """A highlighted progress line (generation started / finished)."""
    print(f"{COLOR_BLUE}[GA] {message}{COLOR_RESET}", flush=True)


def format_duration(seconds: Optional[float]) -> str:
    """Human-readable duration: "1h 05m", "4m 10s", "12s" -- or "--" when
    unknown (None, NaN or negative)."""
    if seconds is None or seconds != seconds or seconds < 0:   # seconds != seconds is the NaN test
        return "--"
    hours, rest = divmod(int(round(seconds)), 3600)
    minutes, secs = divmod(rest, 60)
    if hours:
        return f"{hours}h {minutes:02d}m"
    return f"{minutes}m {secs:02d}s" if minutes else f"{secs}s"


# =============================================================================
# JSON files
# =============================================================================
def write_json(path: Path, data: Any) -> None:
    """Write JSON atomically, so a reader (the GUI) never sees half a file."""
    write_atomic(path, json.dumps(data, indent=2, default=str).encode())


def read_json(path: Path) -> Optional[dict]:
    """Read a JSON file, or None if it's missing or not (yet) valid JSON."""
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


# =============================================================================
# The session folder
# =============================================================================
# history.csv columns, before the gene columns (one per gene, in physical
# units, named after the gene).
HISTORY_COLUMNS = [
    "eval_id", "generation", "status", "run_dir",
    "position_rms", "velocity_rms", "effort_rms", "saturation_fraction",
    "final_time", "crash_time", "crash_reason",
    "obj_position", "obj_velocity", "obj_effort", "weighted",
]


class SessionFiles:
    """The files of one session folder. Creating it creates the folder and
    writes history.csv's header."""

    def __init__(self, session_dir: Path, gene_names: list[str]) -> None:
        self.session_dir = session_dir
        self.console_dir = session_dir / "console"
        self.pareto_dir = session_dir / "pareto"
        self.progress_path = session_dir / "progress.json"
        self.history_path = session_dir / "history.csv"
        self.pareto_path = session_dir / "pareto.json"
        self.console_dir.mkdir(parents=True, exist_ok=True)

        # progress.json is rewritten whole on every update; this holds its
        # current contents so an update only has to pass what changed.
        self._progress: dict[str, Any] = {"state": "starting", "started": time.strftime("%Y-%m-%d %H:%M:%S")}

        with self.history_path.open("w", newline="") as handle:
            csv.writer(handle).writerow(HISTORY_COLUMNS + gene_names)

    def update_progress(self, **fields: Any) -> None:
        self._progress.update(fields, updated=time.strftime("%Y-%m-%d %H:%M:%S"))
        write_json(self.progress_path, self._progress)

    def append_history(self, row: dict[str, Any], genes: list[float]) -> None:
        """Append one candidate. Columns missing from `row` are left empty."""
        with self.history_path.open("a", newline="") as handle:
            csv.writer(handle).writerow([row.get(name, "") for name in HISTORY_COLUMNS] + genes)
