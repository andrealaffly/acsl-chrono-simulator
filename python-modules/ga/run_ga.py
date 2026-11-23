#!/usr/bin/env python3
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
# File:        run_ga.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Command-line entry point of the GA tuner. It tunes the gains of the
#     platform/controller that is active in config/sim-config.yaml.
#
#     Usage (from anywhere):
#         python3 python-modules/ga/run_ga.py [--session FILE]   run a session
#         python3 python-modules/ga/run_ga.py --list             list tunable gains
#         python3 python-modules/ga/run_ga.py --check            pre-flight only
#     FILE is merged over config/ga-config.yaml (the GUI writes one per Start).
#
#     This file only handles the command line and the pre-flight checks;
#     the session itself is session.run_session(). Exit codes are listed in
#     session.py (EXIT_*).
###############################################################################

from __future__ import annotations

import sys
from pathlib import Path

# Run as a script, Python puts this file's folder (python-modules/ga/) on the
# import path, which would make the package's modules importable as
# top-level modules ("import metrics"). Replace it with python-modules/, so
# the package is imported as "ga" like everywhere else.
if not __package__:
    sys.path[0] = str(Path(__file__).resolve().parent.parent)

import argparse
import json

from ga import preflight, reporting
from ga.context import ConfigError, SimContext, load_ga_config
from ga.gains_codec import MODE_DESCRIPTIONS, discover_entries
from ga.session import EXIT_DONE, EXIT_NOT_STARTED, run_session


def _list_entries(ctx: SimContext, config: dict) -> None:
    """--list: print every entry of the active gains file and the modes it allows."""
    reporting.info(f"TUNABLE ENTRIES OF {ctx.gains_path}")
    for entry in discover_entries(json.loads(ctx.gains_path.read_text()), config.get("exclude_patterns") or []):
        if entry.excluded:
            modes = "excluded"
        elif entry.modes:
            modes = ", ".join(f"{mode} ({MODE_DESCRIPTIONS[mode]})" for mode in entry.modes)
        else:
            modes = entry.note
        print(f"  {entry.path:<48} {entry.summary:<14} {modes}", flush=True)


def _report_checks(checks: list[preflight.Check]) -> None:
    """Print the pre-flight checks: OK, FAIL (blocking) or WARN."""
    for check in checks:
        text = check.label + (f": {check.detail}" if check.detail else "")
        if check.passed:
            reporting.info(f"[ OK ] {text}")
        elif check.blocking:
            reporting.error(f"[FAIL] {text}")
        else:
            reporting.warning(f"[WARN] {text}")


def main() -> int:
    parser = argparse.ArgumentParser(description="GA tuning of the active controller's gains.")
    parser.add_argument("--session", type=Path, help="session YAML merged over config/ga-config.yaml")
    parser.add_argument("--list", action="store_true", help="list the active controller's tunable gains and exit")
    parser.add_argument("--check", action="store_true", help="run the pre-flight checks and exit")
    args = parser.parse_args()

    try:
        config = load_ga_config(args.session)
    except ConfigError as error:
        reporting.error(str(error))
        return EXIT_NOT_STARTED

    ctx, checks = preflight.run_preflight(config)

    # --list only needs the gains file; if it can't be found, the checks
    # below say why.
    if args.list and ctx is not None and ctx.gains_path.is_file():
        _list_entries(ctx, config)
        return EXIT_DONE

    _report_checks(checks)
    if args.list or ctx is None or preflight.blocking_failures(checks):
        reporting.error("PRE-FLIGHT FAILED -- fix the items above (Settings page or config/*.yaml) and try again")
        return EXIT_NOT_STARTED
    if args.check:
        reporting.info("PRE-FLIGHT PASSED")
        return EXIT_DONE

    return run_session(config, ctx)


if __name__ == "__main__":
    sys.exit(main())
