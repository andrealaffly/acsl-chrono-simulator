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
# File:        context.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Where everything is. This module answers three questions for the rest
#     of the package:
#
#       1. Where are the repo's folders?  (REPO_DIR, CONFIG_DIR, BUILD_DIR, ...)
#       2. What is being tuned?           (resolve_context -> SimContext)
#          The active platform and controller are read from
#          config/sim-config.yaml exactly the way the simulator reads them,
#          so the GA always tunes whatever the simulator would fly.
#       3. How is the GA configured?      (load_ga_config)
#          config/ga-config.yaml, with an optional session file merged on
#          top (the GUI writes one for every Start).
#
#     Nothing here calls sys.exit(): the GUI imports this module into its own
#     process, so every problem is raised as a ConfigError for the caller to
#     show however it likes.
###############################################################################

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import yaml

from .utils import deep_merge

# =============================================================================
# Repo layout
# =============================================================================
# This file lives at <repo>/python-modules/ga/context.py, so the repo root is
# two levels above the package folder.
PACKAGE_DIR = Path(__file__).resolve().parent
REPO_DIR = PACKAGE_DIR.parent.parent

CONFIG_DIR = REPO_DIR / "config"                          # the active configuration
BUILD_DIR = REPO_DIR / "build"                            # where CMake builds the simulator
EXECUTABLE = BUILD_DIR / "acsl_sim"                       # the simulator itself
SIM_LOG_DIR = REPO_DIR / "sim-log"                        # every run's logs
PARAMETERS_DIR = REPO_DIR / "chrono-assets" / "parameters"  # every controller's gains

SIM_CONFIG_PATH = CONFIG_DIR / "sim-config.yaml"
VIS_CONFIG_PATH = CONFIG_DIR / "vis-config.yaml"
GA_CONFIG_PATH = CONFIG_DIR / "ga-config.yaml"

# Folders created next to gains_<CTRL>.json:
#   baseline_tune/  a copy of the starting gains, taken when a session begins
#   ga_tuned/       the best gains a session found
BASELINE_FOLDER = "baseline_tune"
TUNED_FOLDER = "ga_tuned"


class ConfigError(Exception):
    """A config file is missing, can't be parsed, or selects an invalid setup."""


def load_yaml(path: Path) -> dict:
    """Read a YAML file into a dictionary (an empty file gives {})."""
    if not path.is_file():
        raise ConfigError(f"Config file not found: {path}")
    try:
        with path.open("r") as handle:
            return yaml.safe_load(handle) or {}
    except yaml.YAMLError as error:
        raise ConfigError(f"Could not parse {path}: {error}") from error


# =============================================================================
# The active platform / controller
# =============================================================================
def gains_path_for(platform: str, controller_key: str) -> Path:
    """The gains file the simulator reads for a platform/controller pair.

    The C++ side names a controller's parameter and log folders after its
    sim-config key in upper case (mrac_geometric -> MRAC_GEOMETRIC), and every
    controller reads <platform>/<CTRL>/gains_<CTRL>.json. The session's first
    dry run checks this against the folder the simulator really logs to, so
    a controller that breaks the convention fails loudly, not silently.
    """
    name = controller_key.upper()
    return PARAMETERS_DIR / platform / name / f"gains_{name}.json"


def _exclusive_selection(group: Any, description: str) -> str:
    """Return the one key of `group` that is set to true.

    sim-config.yaml selects the platform (and the controller of each
    platform) with a group of booleans of which exactly one must be true.
    This mirrors validateExclusiveSelection on the C++ side.
    """
    if not isinstance(group, dict):
        raise ConfigError(f"sim-config.yaml: missing '{description}' section")
    active = [name for name, enabled in group.items() if enabled is True]
    if len(active) != 1:
        raise ConfigError(f"sim-config.yaml: expected exactly one active {description}, found {active or 'none'}")
    return active[0]


@dataclass
class SimContext:
    """What is being tuned, and every path that follows from it."""

    platform: str          # e.g. "x8"
    controller_key: str    # sim-config key, e.g. "mrac_geometric"
    sim_config: dict       # config/sim-config.yaml, as read
    vis_config: dict       # config/vis-config.yaml, as read

    @property
    def gains_path(self) -> Path:
        """The gains file the simulator reads (and the GA rewrites per candidate)."""
        return gains_path_for(self.platform, self.controller_key)

    @property
    def controller_name(self) -> str:
        """Folder name of the controller, e.g. "MRAC_GEOMETRIC"."""
        return self.gains_path.parent.name

    @property
    def controller_dir(self) -> Path:
        return self.gains_path.parent

    @property
    def baseline_dir(self) -> Path:
        return self.controller_dir / BASELINE_FOLDER

    @property
    def tuned_dir(self) -> Path:
        return self.controller_dir / TUNED_FOLDER

    @property
    def wrapper_log_dir(self) -> Path:
        """Where wrapper-mode runs log: sim-log/<platform>/wrapper/<date>/run_N/."""
        return SIM_LOG_DIR / self.platform / "wrapper"

    @property
    def ga_log_dir(self) -> Path:
        """Where GA sessions write their results: sim-log/<platform>/ga/<timestamp>/."""
        return SIM_LOG_DIR / self.platform / "ga"

    @property
    def label(self) -> str:
        return f"{self.platform} / {self.controller_name}"


def resolve_context() -> SimContext:
    """Read sim-config.yaml / vis-config.yaml and return what is being tuned."""
    sim_config = load_yaml(SIM_CONFIG_PATH)
    platform = _exclusive_selection(sim_config.get("platform"), "platform")
    controllers = (sim_config.get("controller") or {}).get(platform)
    controller_key = _exclusive_selection(controllers, f"controller for platform '{platform}'")
    return SimContext(platform, controller_key, sim_config, load_yaml(VIS_CONFIG_PATH))


# =============================================================================
# The GA config
# =============================================================================
def load_ga_config(session_path: Optional[Path] = None) -> dict:
    """config/ga-config.yaml, with the session file (if any) merged on top.

    A session file only has to list what it changes -- usually just "tune"
    and a few settings -- because everything else comes from ga-config.yaml.
    """
    config = load_yaml(GA_CONFIG_PATH)
    if session_path is not None:
        config = deep_merge(config, load_yaml(session_path))
    return config
