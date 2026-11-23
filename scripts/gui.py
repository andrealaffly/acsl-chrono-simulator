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
# File:        gui.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Entry point for the ACSL UAV Simulator control panel GUI. Launches the
#     application shell (scripts/acsl_gui/shell.py) with the pages registered
#     below. Today that is just the Settings page (a comment-preserving
#     editor for config/*.yaml); Run/Build/Clean/wrapper pages are meant to
#     be added to the PAGES list here as they're written, with no changes
#     needed to the shell itself.
#
#     Run from anywhere with:
#         python3 scripts/gui.py
###############################################################################

#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parent
CONFIG_DIR = REPO_DIR / "config"

# The GA tuner lives in python-modules/ga (imported as "ga" by the Genetic
# Algorithm page), outside scripts/, so its folder goes on the import path.
PYTHON_MODULES_DIR = REPO_DIR / "python-modules"
if str(PYTHON_MODULES_DIR) not in sys.path:
    sys.path.insert(0, str(PYTHON_MODULES_DIR))


# Checked before anything else imports tkinter, so a missing system package
# fails with a one-line fix instead of a raw ModuleNotFoundError traceback.
def _require_tkinter() -> None:
    if importlib.util.find_spec("tkinter") is not None:
        return
    print(
        "Tkinter is required but not installed.\n\n"
        "Install it with:\n\n"
        "    sudo apt install python3-tk\n",
        file=sys.stderr,
    )
    sys.exit(1)


def main() -> int:
    _require_tkinter()

    # Imported here, not at module scope, so _require_tkinter()'s friendly
    # error above is what a missing tkinter shows -- these all pull in
    # tkinter transitively, and a top-level import would fail before that
    # check ever ran.
    from acsl_gui.pages.settings import SettingsPage
    from acsl_gui.pages.simulator import SimulatorPage
    from acsl_gui.shell import AppShell

    # The GA page needs numpy (and the GA tuner package); without them the
    # rest of the GUI still works, just without that page.
    try:
        from acsl_gui.pages.genetic_algorithm import GeneticAlgorithmPage
    except ImportError as error:
        GeneticAlgorithmPage = None
        print(f"Genetic Algorithm page disabled ({error}). Install numpy and pymoo to enable it.", file=sys.stderr)

    if not CONFIG_DIR.is_dir():
        print(f"Config directory not found: {CONFIG_DIR}", file=sys.stderr)
        return 1

    shell = AppShell(repo_dir=REPO_DIR)
    # Pages hold a reference back to the shell (for shared status/navigation),
    # so they're constructed after it exists, then registered with it. Order
    # here is sidebar order. Add future pages (a flightstack wrapper, ...) to
    # this list -- nothing else here or in shell.py needs to change.
    pages = [SimulatorPage(shell, REPO_DIR)]
    if GeneticAlgorithmPage is not None:
        pages.append(GeneticAlgorithmPage(shell, REPO_DIR))
    pages.append(SettingsPage(shell, REPO_DIR))
    shell.register_pages(pages)
    shell.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
