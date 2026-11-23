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
# File:        sim_runner.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Runs a batch of candidates through build/acsl_sim, the same way
#     scripts/wrapper.py runs a session: up to "concurrent_runs" simulations
#     at once, launched one after another, each with its console output in
#     its own file. The simulator runs in wrapper mode, so every run logs to
#     its own folder sim-log/<platform>/wrapper/<date>/run_N/.
#
#     THE SHARED GAINS FILE
#     Unlike wrapper.py, every run here needs DIFFERENT gains, but they all
#     read them from the SAME file. So candidate B's gains must not be
#     written until candidate A has finished reading its own. A run reads
#     the file only once Chrono has finished setting up, which can take
#     longer than any fixed delay, so instead:
#
#       1. write candidate A's gains, launch A;
#       2. wait until A has copied its gains into its log folder
#          (run_N/<CTRL>/params/gains_<CTRL>.json) -- the controller does
#          that right after reading them -- and at least stagger_seconds
#          have passed;
#       3. only then write candidate B's gains and launch B; and so on.
#
#     Only one run is ever "starting" (gains written but not yet read). The
#     run_N folder that appears in step 2 is also how each result is matched
#     to its candidate, and once a run finishes, its copied gains are
#     compared byte for byte with what was written for it (gains_match), so
#     a mix-up can never go unnoticed.
###############################################################################

from __future__ import annotations

import signal
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Callable, Optional

from . import reporting
from .context import BUILD_DIR, EXECUTABLE, SimContext

# How often the batch loop checks on its runs.
POLL_SECONDS = 0.2


@dataclass
class RunRequest:
    """One simulation to run."""

    key: str        # unique within a batch; results are returned by it
    label: str      # for console messages
    payload: bytes  # the gains file contents to run with


@dataclass
class RunResult:
    """How one simulation went."""

    key: str
    run_dir: Optional[Path]         # sim-log/.../run_N, if the run got that far
    controller_dir: Optional[Path]  # run_N/<CTRL>, where controller_log.log is
    returncode: Optional[int]
    error: Optional[str]            # None if the run started and finished normally
    gains_match: bool               # the run's copied gains == the payload
    duration: float                 # seconds from launch to finish
    console_log: Path               # the run's stdout/stderr


@dataclass
class _Run:
    """Bookkeeping for a run in progress."""

    request: RunRequest
    process: subprocess.Popen
    log_file: BinaryIO
    console_log: Path
    started_at: float
    known_dirs: set[Path]                   # run_N folders that existed before this launch
    run_dir: Optional[Path] = None          # set once this run's folder appears
    controller_dir: Optional[Path] = None


@dataclass
class RunnerSettings:
    """ga-config.yaml "runner" settings (see that file for what each means)."""

    concurrent_runs: int
    stagger_seconds: float
    start_timeout_seconds: float
    run_timeout_seconds: float   # 0 = no limit

    @classmethod
    def from_config(cls, config: dict) -> "RunnerSettings":
        runner = config.get("runner") or {}
        return cls(
            concurrent_runs=max(1, int(runner.get("concurrent_runs", 1))),
            stagger_seconds=max(0.0, float(runner.get("stagger_seconds", 1.0))),
            start_timeout_seconds=max(1.0, float(runner.get("start_timeout_seconds", 180.0))),
            run_timeout_seconds=max(0.0, float(runner.get("run_timeout_seconds", 0.0))),
        )


class _LaunchFailed(Exception):
    """A run died, or timed out, before reading its gains."""


def _stop(process: subprocess.Popen) -> None:
    """Ask a run to stop (SIGINT, like Ctrl+C); kill it if it hasn't
    within five seconds."""
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


class SimRunner:
    def __init__(self, ctx: SimContext, settings: RunnerSettings, console_dir: Path, write_gains: Callable[[bytes], None]) -> None:
        """
        console_dir  where each run's console output goes (<key>.log)
        write_gains  puts a payload where the simulator reads it
                     (GainsStore.write_candidate)
        """
        self.ctx = ctx
        self.settings = settings
        self.console_dir = console_dir
        self.write_gains = write_gains
        self._active: dict[str, _Run] = {}    # runs launched and not yet finished, by key
        self._last_launch: Optional[float] = None

    def _run_dirs(self) -> set[Path]:
        """Every run_N folder currently under sim-log/<platform>/wrapper/."""
        return {path for path in self.ctx.wrapper_log_dir.glob("*/run_*") if path.is_dir()}

    @staticmethod
    def _copied_gains(run_dir: Path) -> Optional[Path]:
        """The gains file a run copied into its log folder, if it has yet."""
        return next(iter(sorted(run_dir.glob("*/params/gains_*.json"))), None)

    def _launch(self, request: RunRequest) -> _Run:
        """Write the candidate's gains and start the simulator, no sooner
        than stagger_seconds after the previous launch."""
        if self._last_launch is not None:
            time.sleep(max(0.0, self.settings.stagger_seconds - (time.monotonic() - self._last_launch)))
        known_dirs = self._run_dirs()   # so the new run's folder can be told apart
        self.write_gains(request.payload)
        console_log = self.console_dir / f"{request.key}.log"
        log_file = console_log.open("wb")
        process = subprocess.Popen([f"./{EXECUTABLE.name}"], cwd=str(BUILD_DIR), stdin=subprocess.DEVNULL, stdout=log_file, stderr=subprocess.STDOUT)
        self._last_launch = time.monotonic()
        reporting.info(f"STARTED {request.label} (PID {process.pid})")
        return _Run(request, process, log_file, console_log, time.monotonic(), known_dirs)

    def _has_loaded_gains(self, run: _Run) -> bool:
        """True once a starting run has copied its gains (so it has read
        them), False while still waiting. Raises _LaunchFailed if it never
        will (it exited, or start_timeout_seconds passed)."""
        for run_dir in sorted(self._run_dirs() - run.known_dirs):
            copied = self._copied_gains(run_dir)
            if copied is not None:
                # copied = run_N/<CTRL>/params/gains_*.json, so its parent's
                # parent is the controller's log folder.
                run.run_dir, run.controller_dir = run_dir, copied.parent.parent
                return True
        if run.process.poll() is not None:
            raise _LaunchFailed(f"exited (code {run.process.returncode}) before loading its gains")
        if time.monotonic() - run.started_at > self.settings.start_timeout_seconds:
            raise _LaunchFailed(f"did not load its gains within {self.settings.start_timeout_seconds:g} s")
        return False

    def _finish(self, run: _Run, error: Optional[str] = None) -> RunResult:
        """Stop a run (if still going), close its console file, and report it."""
        del self._active[run.request.key]
        _stop(run.process)
        run.log_file.close()
        copied = self._copied_gains(run.run_dir) if error is None and run.run_dir is not None else None
        return RunResult(
            key=run.request.key,
            run_dir=run.run_dir,
            controller_dir=run.controller_dir,
            returncode=run.process.returncode,
            error=error,
            gains_match=copied is not None and copied.read_bytes() == run.request.payload,
            duration=time.monotonic() - run.started_at,
            console_log=run.console_log,
        )

    def run_batch(self, requests: list[RunRequest], on_result: Optional[Callable[[RunResult], None]] = None) -> dict[str, RunResult]:
        """Run every request, keeping up to concurrent_runs going at once.

        Returns the results by request key. on_result, if given, is called
        as each run finishes (in finishing order, not request order).
        """
        queue = deque(requests)
        results: dict[str, RunResult] = {}
        # The one run whose gains are written but not yet read (see the file
        # header): no other candidate is written while this is set.
        starting: Optional[_Run] = None

        def done(result: RunResult) -> None:
            results[result.key] = result
            if on_result is not None:
                on_result(result)

        while queue or self._active:
            # 1. Launch the next candidate if nothing is starting and there's a free slot.
            if starting is None and queue and len(self._active) < self.settings.concurrent_runs:
                starting = self._launch(queue.popleft())
                self._active[starting.request.key] = starting

            # 2. Check whether the starting run has read its gains yet.
            if starting is not None:
                try:
                    if self._has_loaded_gains(starting):
                        starting = None
                except _LaunchFailed as failure:
                    reporting.warning(f"{starting.request.label} {failure}")
                    done(self._finish(starting, str(failure)))
                    starting = None

            # 3. Collect runs that have exited, and stop runs over their time limit.
            for run in [run for run in self._active.values() if run is not starting]:
                elapsed = time.monotonic() - run.started_at
                if run.process.poll() is not None:
                    done(self._finish(run))
                elif self.settings.run_timeout_seconds and elapsed > self.settings.run_timeout_seconds:
                    reporting.warning(f"{run.request.label} timed out after {elapsed:0.0f} s, stopping it")
                    done(self._finish(run, f"timed out after {elapsed:0.0f} s"))

            time.sleep(POLL_SECONDS)
        return results

    def terminate_all(self) -> None:
        """Stop every run still going (used when the session ends early).
        All of them are signalled first, then waited for, so they wind down
        in parallel."""
        for run in self._active.values():
            if run.process.poll() is None:
                run.process.send_signal(signal.SIGINT)
        for run in list(self._active.values()):
            _stop(run.process)
            run.log_file.close()
        self._active.clear()


def deactivate_wrapper_session_marker(ctx: SimContext) -> None:
    """Mark the wrapper session as finished.

    In wrapper mode the simulator groups runs into one dated folder for as
    long as sim-log/<platform>/wrapper/.session.bak says "active". The
    simulator itself can't know when a whole session is over, so whoever
    drives the runs (scripts/wrapper.py, or here, the GA) writes "inactive"
    when done. Then the next session starts a fresh folder.
    """
    marker = ctx.wrapper_log_dir / ".session.bak"
    if not marker.parent.is_dir():
        return  # no run ever got far enough to create the wrapper folder
    try:
        marker.write_text("inactive\n")
        reporting.info(f"WRAPPER SESSION MARKED INACTIVE: {marker}")
    except OSError as error:
        reporting.warning(f"COULD NOT WRITE SESSION MARKER INACTIVE: {marker} ({error})")
