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
# File:        preflight.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Checks that the simulator is set up for GA tuning before a session
#     starts. Used by run_ga.py (which refuses to start on a blocking
#     failure) and by the GUI's GA page (which lists every check).
#
#     Nothing here changes a config file: a failed check says what to change,
#     and the GUI's Settings page is where to change it.
#
#     WHY THESE REQUIREMENTS
#     Comparing candidates is only fair if every run is identical except for
#     its gains. So the simulator must run:
#       - headless: no visualization, no per-step terminal output;
#       - in wrapper mode, so every run gets its own run_N log folder;
#       - from the origin, with nothing randomized;
#       - with no injected motor failure;
#       - over the full trajectory, with the controller actually in the loop
#         (no debug stop, developer mode, rigid-body mode or SIL/HIL loop).
###############################################################################

from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from typing import Optional

from . import gains_store
from .context import EXECUTABLE, SIM_LOG_DIR, ConfigError, SimContext, resolve_context
from .log_reader import ColumnSpec, LogError, check_columns, latest_controller_log, read_header
from .optimizer import ALGORITHMS
from .utils import get_dotted


@dataclass
class Check:
    """One pre-flight check."""

    label: str
    passed: bool
    detail: str = ""   # why it failed (or, for some passing checks, what was found)
    # A failed blocking check stops a session from starting; a failed
    # non-blocking one is only a warning.
    blocking: bool = True


# The config values the GA needs: (config file, dotted key, required value, label).
_REQUIREMENTS = [
    ("sim-config.yaml", "mode.enable_wrapper", True, "Wrapper mode on"),
    ("sim-config.yaml", "mode.enable_flightstack_loop", False, "SIL/HIL flightstack loop off"),
    ("sim-config.yaml", "debug.terminal", False, "Terminal state printing off"),
    ("sim-config.yaml", "debug.sim_debug_stop", False, "Debug stop time off (full trajectory)"),
    ("sim-config.yaml", "debug.developer_mode", False, "Developer mode off"),
    ("sim-config.yaml", "debug.developer_mode_steps", False, "Developer step mode off"),
    ("sim-config.yaml", "debug.treat_as_rigid_body", False, "Rigid-body mode off"),
    ("sim-config.yaml", "origin.set_origin_to_zero", True, "Origin at (0, 0, 0)"),
    ("sim-config.yaml", "origin.set_origin_to_custom", False, "Custom origin off"),
    ("sim-config.yaml", "origin.set_origin_to_random", False, "Random origin off"),
    ("sim-config.yaml", "motor_disturbances.reduce_efficiency", False, "Motor disturbance off"),
    ("vis-config.yaml", "main.enable_vis", False, "Visualization off"),
]


def _check(label: str, passed: bool, failure: str, blocking: bool = True) -> Check:
    """A Check whose detail is `failure` only if it failed."""
    return Check(label, passed, "" if passed else failure, blocking)


def config_checks(ga_config: dict) -> list[Check]:
    """Checks on the GA config itself."""
    name = str(get_dotted(ga_config, "algorithm.name"))
    try:
        ColumnSpec.from_config(ga_config)
        column_problem = ""
    except ValueError as error:
        column_problem = str(error)
    weights = get_dotted(ga_config, "objectives.weights")
    positive = isinstance(weights, dict) and any(float(value) > 0 for value in weights.values())
    return [
        _check("GA algorithm", name in ALGORITHMS, f"unknown algorithm '{name}' (choose from {', '.join(ALGORITHMS)})"),
        _check("Column contract in GA config", not column_problem, column_problem),
        _check("Objective weights", positive, "at least one weight must be positive"),
    ]


def column_check(ctx: SimContext, ga_config: dict) -> Check:
    """Check the header of this controller's most recent log, if any.

    Only ever a warning: the log may predate a rebuild, and the session's
    baseline dry run re-checks the columns on a fresh log before anything
    is tuned.
    """
    label = "Controller log columns"
    log_path = latest_controller_log(SIM_LOG_DIR / ctx.platform, ctx.controller_name)
    if log_path is None:
        return Check(label, True, "no log of this controller yet; checked during the dry run", blocking=False)
    try:
        result = check_columns(read_header(log_path), ColumnSpec.from_config(ga_config))
    except (LogError, ValueError) as error:
        return Check(label, False, str(error), blocking=False)
    where = log_path.relative_to(SIM_LOG_DIR)
    if result.ok:
        return Check(label, True, f"{result.describe()} in {where}", blocking=False)
    return Check(label, False, f"{result.describe()} in {where} (re-checked during the dry run)", blocking=False)


def run_preflight(ga_config: dict) -> tuple[Optional[SimContext], list[Check]]:
    """Run every check. Returns the resolved SimContext (None if the
    platform/controller selection itself is broken) and the checks, in the
    order they should be shown."""
    has_pymoo = importlib.util.find_spec("pymoo") is not None
    checks = [_check("pymoo installed", has_pymoo, "install it with scripts/installer.py, or: pip3 install --user pymoo")]
    checks += config_checks(ga_config)

    try:
        ctx = resolve_context()
    except ConfigError as error:
        # Without a platform/controller nothing else can be checked.
        return None, checks + [Check("Platform and controller selection", False, str(error))]
    checks.append(Check("Platform and controller selection", True, ctx.label))

    checks.append(_check("Simulator executable", EXECUTABLE.is_file(), f"{EXECUTABLE} not found (build the simulator first)"))
    checks.append(_check("Gains file", ctx.gains_path.is_file(), f"{ctx.gains_path} not found"))
    configs = {"sim-config.yaml": ctx.sim_config, "vis-config.yaml": ctx.vis_config}
    for filename, key, required, label in _REQUIREMENTS:
        value = get_dotted(configs[filename], key)
        checks.append(_check(label, value is required, f"set {key} to {str(required).lower()} in {filename} (currently {value})"))

    running = gains_store.active_session_pid()
    checks.append(_check("No other GA session running", running is None, f"GA session PID {running} is still running"))
    checks.append(column_check(ctx, ga_config))
    return ctx, checks


def blocking_failures(checks: list[Check]) -> list[Check]:
    """The checks that stop a session from starting."""
    return [check for check in checks if check.blocking and not check.passed]
