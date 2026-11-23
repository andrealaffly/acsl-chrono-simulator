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
# File:        __init__.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     The GA tuner: tunes the gains of the active controller by running the
#     simulator over and over with candidate gains, scoring each run from
#     its controller log, and letting an evolutionary optimizer (pymoo)
#     propose better candidates.
#
#     HOW TO RUN IT
#       python3 python-modules/ga/run_ga.py --list     what can be tuned
#       python3 python-modules/ga/run_ga.py --check    is the setup GA-ready
#       python3 python-modules/ga/run_ga.py --session FILE
#     or from the GUI's Genetic Algorithm page (scripts/gui.py), which
#     builds FILE from its Gains and Settings tabs and runs the above.
#     Defaults live in config/ga-config.yaml.
#
#     THE BIG PICTURE
#
#       config/sim-config.yaml ─┐
#       config/ga-config.yaml ──┤ context.py: what to tune, where things are
#                               ▼
#       gains_<CTRL>.json ──► gains_codec.py ──► gene vector x  (log space)
#                                    ▲                │
#                                    │ decode(x)      ▼
#                                    │          optimizer.py (pymoo)
#                                    │          ask() -> generation of x's
#                                    │          tell(scores)
#                                    ▼                ▲
#       gains_store.py: write candidate over the      │
#       active gains file (snapshot + restore)        │
#                                    │                │
#                                    ▼                │
#       sim_runner.py: build/acsl_sim, N in parallel  │
#                                    │                │
#                                    ▼                │
#       log_reader.py + metrics.py: controller_log ──►┘
#       -> position / velocity / effort, crash penalty
#
#       session.py orchestrates all of that (dry run, generations, cleanup,
#       results); reporting.py prints the console lines and writes the
#       session folder; preflight.py checks the setup before anything runs.
#
#     MODULES
#       run_ga.py      command line (--list / --check / --session)
#       session.py     one session from start to finish; exit codes
#       context.py     repo paths, the active platform/controller, GA config
#       gains_codec.py gains JSON <-> gene vector
#       optimizer.py   pymoo ask/tell wrapper (NSGA-II, DE, GA)
#       sim_runner.py  launches and watches simulator runs
#       gains_store.py snapshot / lock / restore of the active gains file
#       log_reader.py  controller_log.log parsing and the column contract
#       metrics.py     scores and crash detection
#       preflight.py   setup checks
#       reporting.py   console output and session files
#       gui_api.py     everything the GUI page needs (no tkinter here)
#       utils.py       dotted-path helpers for nested dicts
#
#     IMPORTING
#     The package is imported as "ga", with python-modules/ on the import
#     path (scripts/gui.py and run_ga.py add it). Nothing in it imports
#     tkinter, and nothing in it imports from scripts/.
###############################################################################
