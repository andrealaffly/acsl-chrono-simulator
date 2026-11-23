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
# File:        gui_api.py
# Authors:     Giri M. Kumar
# Date:        October 26, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Everything the GUI's Genetic Algorithm page
#     (scripts/acsl_gui/pages/genetic_algorithm.py) needs from the GA, in
#     one place and without any tkinter. The page imports only this module;
#     it draws widgets and calls these functions, and all of the GA logic
#     stays here in python-modules/ga.
#
#     How the page and the GA talk to each other:
#
#       GUI page                               GA (this package)
#       --------                               -----------------
#       Settings tab         -- edits config/ga-config.yaml in place
#                               (display_text / file_text / check_setting)
#       Gains tab            -- build_session() --> session dict ("tune")
#       Start                -- write_session_file() + ga_command() -->
#                               run_ga.py, in a separate process
#       Run tab (polling)    <-- read_progress() / read_pareto() --
#                               progress.json / pareto.json in the
#                               session folder (find_session_dir())
#       Stop                 -- SIGINT to that process (run_ga.py then
#                               restores the gains and exits EXIT_STOPPED)
#       Exit                 <-- exit code -> exit_message()
#       Apply                -- resolve_apply_source() / saved_gains_files()
#                               + apply_gains_file()
#
#     The GA runs as a separate process (not a thread) so the GUI stays
#     responsive and a crash in either can't take down the other.
###############################################################################

from __future__ import annotations

import sys
from pathlib import Path
from typing import Iterable, Optional

import yaml

# Re-exported for the page, so it never imports the rest of the package.
from .context import GA_CONFIG_PATH, REPO_DIR, SIM_LOG_DIR, ConfigError, SimContext, load_ga_config, resolve_context  # noqa: F401
from .gains_codec import MODE_DESCRIPTIONS, Entry, GainsCodec, Selection, discover_entries  # noqa: F401
from .gains_store import StoreError, apply_gains_file, recover_stale_lock  # noqa: F401
from .log_reader import latest_controller_log, wall_clock_duration
from .optimizer import ALGORITHMS  # noqa: F401
from .preflight import Check, blocking_failures, column_check, run_preflight  # noqa: F401
from .reporting import format_duration, read_json  # noqa: F401
from .session import EXIT_DONE, EXIT_NOT_STARTED, EXIT_STOPPED

# The command-line entry point the page launches.
RUN_GA_SCRIPT = Path(__file__).resolve().parent / "run_ga.py"

# The mode shown for an entry that isn't being tuned.
OFF = "off"


# =============================================================================
# Settings tab (edits config/ga-config.yaml in place)
# =============================================================================
# The Settings tab is an editor for config/ga-config.yaml, the same way the
# GUI's Settings page edits sim-config.yaml and friends: every scalar value in
# the file is shown, with the file's own comments as help text, and Save
# writes the changes back without touching the comments. These are the GA's
# rules for that editor -- what may be edited, how a value is shown, how it
# is written back, and what counts as valid.

# Values the Settings tab doesn't show: "tune" is built by the Gains tab for
# every Start. (List values, such as the column names and exclude_patterns,
# can't be shown by the editor at all; they're edited in the file itself.)
NOT_EDITABLE = {"tune"}

# Limits on numeric values: dotted key -> (int or float, minimum, strict).
# strict=True means the value must be greater than the minimum, not equal.
# Keys not listed only have to be a valid number / string for their type.
SETTING_LIMITS: dict[str, tuple[type, float, bool]] = {
    "algorithm.population_size": (int, 4, False),
    "algorithm.generations": (int, 1, False),
    "objectives.weights.position": (float, 0, False),
    "objectives.weights.velocity": (float, 0, False),
    "objectives.weights.effort": (float, 0, False),
    "objectives.max_normalized": (float, 0, True),
    "objectives.ignore_initial_seconds": (float, 0, False),
    "objectives.saturation_low": (float, 0, False),
    "objectives.saturation_high": (float, 0, False),
    "crash.position_error_threshold": (float, 0, True),
    "crash.hold_seconds": (float, 0, False),
    "crash.penalty_base": (float, 0, True),
    "crash.time_tolerance_seconds": (float, 0, False),
    "runner.concurrent_runs": (int, 1, False),
    "runner.stagger_seconds": (float, 0, False),
    "runner.start_timeout_seconds": (float, 1, False),
    "runner.run_timeout_seconds": (float, 0, False),
    "runner.max_retries": (int, 0, False),
    "runner.repeatability_tolerance": (float, 0, False),
    "default_bound_factor": (float, 1, True),
}

# The random seed: an integer, or null in the file (shown as an empty field).
SEED_KEY = "algorithm.seed"
# The algorithm: a key of ALGORITHMS in the file, its label on screen.
ALGORITHM_KEY = "algorithm.name"


def load_defaults() -> dict:
    """config/ga-config.yaml as a dict (raises ConfigError if missing or broken)."""
    return load_ga_config()


def display_text(key: str, file_text: str) -> str:
    """How a value from the file is shown in its field."""
    if key == ALGORITHM_KEY:
        return ALGORITHMS.get(file_text, file_text)
    if key == SEED_KEY and file_text.lower() in ("null", "~", ""):
        return ""
    return file_text


def file_text(key: str, shown_text: str) -> str:
    """How a field's text is written back to the file (the inverse of
    display_text)."""
    text = shown_text.strip()
    if key == ALGORITHM_KEY:
        return next((name for name, label in ALGORITHMS.items() if text in (name, label)), text)
    if key == SEED_KEY and not text:
        return "null"
    return text


def check_setting(key: str, shown_text: str) -> Optional[str]:
    """What's wrong with a field's text, or None if it's fine. Only the GA's
    own rules are checked here; the editor checks that numbers are numbers."""
    text = shown_text.strip()
    if key == ALGORITHM_KEY:
        return None if file_text(key, text) in ALGORITHMS else f"unknown algorithm '{text}'"
    if key == SEED_KEY:
        if not text:
            return None
        try:
            int(text)
        except ValueError:
            return "must be a whole number, or empty for a random seed"
        return None
    if key not in SETTING_LIMITS:
        return None
    kind, minimum, strict = SETTING_LIMITS[key]
    try:
        value = kind(text)
    except ValueError:
        return f"must be {'a whole number' if kind is int else 'a number'}"
    if value < minimum or (strict and value == minimum):
        return f"must be {'greater than' if strict else 'at least'} {minimum:g}"
    return None


# =============================================================================
# Gains tab
# =============================================================================
def parse_selection(path: str, mode: str, factor_text: str) -> Optional[Selection]:
    """One Gains-tab row as a Selection, or None if its mode is OFF.
    Raises ValueError if the range factor isn't a number greater than 1."""
    if mode == OFF:
        return None
    try:
        factor = float(factor_text)
    except ValueError:
        raise ValueError(f"{path}: range factor '{factor_text}' is not a number")
    if factor <= 1.0:
        raise ValueError(f"{path}: range factor must be greater than 1")
    return Selection(path, mode, factor)


def gene_count(baseline: dict, path: str, mode: str, factor_text: str) -> Optional[int]:
    """How many genes a row adds: 0 when off, None if the row is invalid."""
    try:
        selection = parse_selection(path, mode, factor_text)
    except ValueError:
        return None
    return GainsCodec(baseline, [selection], []).n_var if selection else 0


# =============================================================================
# Start
# =============================================================================
def build_session(rows: Iterable[tuple[str, str, str]]) -> dict:
    """The session dict for one Start: just the "tune" list, from the Gains
    tab's rows, given as (path, mode, range factor text).

    Everything else comes from config/ga-config.yaml, which run_ga.py merges
    this over -- so the Settings tab must be saved before a Start. Raises
    ValueError (with a message fit to show the user) on an invalid row.
    """
    selections = [parse_selection(*row) for row in rows]
    return {"tune": [{"path": s.path, "mode": s.mode, "bound_factor": s.bound_factor} for s in selections if s]}


def write_session_file(ctx: SimContext, session: dict) -> Path:
    """Write the session dict where run_ga.py will read it; returns its path."""
    session_path = ctx.ga_log_dir / ".gui_session.yaml"
    session_path.parent.mkdir(parents=True, exist_ok=True)
    session_path.write_text(yaml.safe_dump(session, sort_keys=False))
    return session_path


def ga_command(session_path: Path) -> list[str]:
    """The command that runs a session (run it with cwd=REPO_DIR)."""
    return [sys.executable, str(RUN_GA_SCRIPT), "--session", str(session_path)]


def previous_run_seconds(ctx: SimContext) -> Optional[float]:
    """Real time the last logged run of this controller took, or None if
    there is no such log. The basis of the runtime estimate."""
    log_path = latest_controller_log(SIM_LOG_DIR / ctx.platform, ctx.controller_name)
    return wall_clock_duration(log_path) if log_path else None


def estimate_runtime(seconds_per_run: float, population_text: str, generations_text: str, concurrent_text: str) -> tuple[int, int, float]:
    """(runs, concurrent runs, estimated total seconds) for a session.

    A session runs 2 baseline dry runs plus population x generations
    candidates, `concurrent` at a time. (Runs in parallel each tend to be a
    bit slower, so this is optimistic.) Raises ValueError if a field isn't
    an integer.
    """
    runs = 2 + int(population_text) * int(generations_text)
    concurrent = max(1, int(concurrent_text))
    return runs, concurrent, seconds_per_run * runs / concurrent


# =============================================================================
# Run tab
# =============================================================================
def find_session_dir(ctx: SimContext, started_at: float) -> Optional[Path]:
    """The session folder run_ga.py created for a Start at `started_at`
    (time.time()): the newest folder with a progress.json written since
    then. None until it exists."""
    if not ctx.ga_log_dir.is_dir():
        return None
    candidates = [path for path in ctx.ga_log_dir.iterdir()
                  if (path / "progress.json").is_file() and path.stat().st_mtime >= started_at - 1]
    return max(candidates, key=lambda path: path.stat().st_mtime, default=None)


def read_progress(session_dir: Path) -> Optional[dict]:
    """progress.json (see GaSession.progress for its keys), or None if not
    written yet."""
    return read_json(session_dir / "progress.json")


def read_pareto(session_dir: Path) -> dict:
    """pareto.json (see GaSession.write_pareto for its keys), or {} if not
    written yet. Its "solutions" are ordered best first."""
    return read_json(session_dir / "pareto.json") or {}


def exit_message(returncode: int) -> str:
    """What a run_ga.py exit code means, for the status bar."""
    return {
        EXIT_DONE: "GA session finished.",
        EXIT_STOPPED: "GA session stopped.",
        EXIT_NOT_STARTED: "GA session did not start (see console).",
    }.get(returncode, f"GA session ended with an error (code {returncode}).")


# =============================================================================
# Apply
# =============================================================================
def resolve_apply_source(pareto: dict, solution: Optional[dict]) -> Optional[Path]:
    """The gains file Apply should copy, without asking the user:

      - a selected solution: its gains file -- or, for the best one, the
        copy saved in ga_tuned/;
      - nothing selected: the session's ga_tuned/ file, if there is one.

    None means "ask the user" (see saved_gains_files).
    """
    tuned = Path(pareto["tuned_file"]) if pareto.get("tuned_file") else None
    if solution is not None:
        if tuned and solution["eval_id"] == pareto.get("best"):
            return tuned
        return Path(solution["gains_file"])
    if tuned and tuned.is_file():
        return tuned
    return None


def saved_gains_files(ctx: SimContext) -> dict[str, Path]:
    """Every gains file saved in ga_tuned/ and baseline_tune/, as
    {"<folder>/<file>": path}, newest first within each folder."""
    files = {}
    for folder in (ctx.tuned_dir, ctx.baseline_dir):
        if not folder.is_dir():
            continue
        for path in sorted(folder.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
            files[f"{folder.name}/{path.name}"] = path
    return files
