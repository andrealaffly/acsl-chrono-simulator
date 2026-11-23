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
# File:        session.py
# Authors:     Giri M. Kumar
# Date:        October 26, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     One GA tuning session, from start to finish. run_session() is the
#     whole thing; GaSession holds its state while it runs.
#
#     THE SESSION, STEP BY STEP
#       1. Setup: recover gains a crashed earlier session left behind, parse
#          the "tune" list into a GainsCodec, create the session folder
#          sim-log/<platform>/ga/<timestamp>/.
#       2. Snapshot the original gains into <CTRL>/baseline_tune/ and take
#          the session lock (GainsStore.begin).
#       3. Dry run (GaSession.dry_run): fly the untouched baseline twice.
#          This checks the log columns and the controller's log folder name,
#          checks the two runs agree (the simulation is deterministic), and
#          makes the first run the reference every candidate is normalized
#          against.
#       4. Optimize (GaSession.optimize): for each generation, ask the
#          optimizer for candidates, simulate them in parallel, score them,
#          and tell the optimizer the scores.
#       5. Clean up, on EVERY exit path -- finished, Ctrl+C / the GUI's Stop,
#          an error, or a bug: stop any simulations still running, restore
#          the original gains file, mark the wrapper session inactive.
#       6. Results (GaSession.finalize): save the best candidate to
#          <CTRL>/ga_tuned/ (if it beat the baseline), and write the Pareto
#          front and the final progress.json.
#
#     CANDIDATE STATUSES (Evaluation.status)
#       ok           flew the whole trajectory; scored normally
#       crashed      crash detected in the log; scored with the penalty
#       ended_early  the log stops before the trajectory ends; penalty
#       failed       no usable log (didn't start, timed out, bad columns)
#       mismatch     the gains the run logged aren't the candidate's;
#                    re-run up to max_retries times, then given up on
###############################################################################

from __future__ import annotations

import json
import signal
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from . import reporting
from .context import SimContext
from .gains_codec import GainsCodec, Selection, dumps
from .gains_store import GainsStore, StoreError, recover_stale_lock
from .log_reader import CONTROLLER_LOG_NAME, ColumnSpec, LogError, check_columns, read_controller_log
from .metrics import OBJECTIVES, MetricSettings, RunMetrics, Scorer, compute_metrics
from .optimizer import ALGORITHMS, Optimizer, is_multi_objective
from .reporting import SessionFiles, format_duration, write_json
from .sim_runner import RunnerSettings, RunRequest, RunResult, SimRunner, deactivate_wrapper_session_marker

# Exit codes of run_ga.py. The GUI's GA page tells the user what happened
# from these, so they are part of the contract between the two.
EXIT_DONE = 0          # finished (or --list / --check succeeded)
EXIT_FAILED = 1        # started, then ended on an error
EXIT_NOT_STARTED = 2   # bad config or failed pre-flight; nothing was touched
EXIT_STOPPED = 130     # stopped by Ctrl+C / SIGTERM (the shell's 128 + SIGINT)


class SessionError(Exception):
    """A problem that ends the session. It is reported, and the gains are
    still restored."""


@dataclass
class Evaluation:
    """One simulated candidate and its scores."""

    eval_id: str                     # "baseline_1", "g003_c007", ...
    generation: int                  # 0 for the baseline dry runs
    x: np.ndarray                    # its genes
    status: str                      # see CANDIDATE STATUSES above
    metrics: Optional[RunMetrics]    # None if the run gave no usable log
    objectives: np.ndarray           # normalized (or penalty) objective vector
    weighted: float                  # weighted sum of the objectives
    run_dir: Optional[Path]
    payload: bytes                   # the gains file it ran with

    def summary(self) -> dict:
        """JSON-friendly summary for progress.json and pareto.json."""
        return {
            "eval_id": self.eval_id,
            "generation": self.generation,
            "status": self.status,
            "weighted": self.weighted,
            "objectives": dict(zip(OBJECTIVES, self.objectives.tolist())),
            "metrics": self.metrics.to_dict() if self.metrics else None,
            "run_dir": str(self.run_dir) if self.run_dir else None,
        }


def _gene_key(x: np.ndarray) -> tuple:
    """A hashable identity for a gene vector. Rounded, so a vector the
    optimizer hands out again matches the one already simulated."""
    return tuple(np.round(np.asarray(x, dtype=float), 12).tolist())


def non_dominated(evaluations: list[Evaluation]) -> list[Evaluation]:
    """The Pareto front: candidates no other candidate beats on every
    objective at once (a "dominates" b if it is no worse on all objectives
    and strictly better on at least one)."""
    def dominates(a: Evaluation, b: Evaluation) -> bool:
        return bool(np.all(a.objectives <= b.objectives) and np.any(a.objectives < b.objectives))
    return [e for e in evaluations if not any(dominates(other, e) for other in evaluations if other is not e)]


# =============================================================================
# GaSession
# =============================================================================
class GaSession:
    def __init__(self, config: dict, ctx: SimContext, codec: GainsCodec, store: GainsStore, files: SessionFiles, runner: SimRunner) -> None:
        self.config = config
        self.ctx = ctx
        self.codec = codec
        self.store = store
        self.files = files
        self.runner = runner
        self.spec = ColumnSpec.from_config(config)
        self.metric_settings = MetricSettings.from_config(config)

        algorithm = config.get("algorithm") or {}
        self.algorithm_name = str(algorithm.get("name", "nsga2")).lower()
        self.population_size = int(algorithm.get("population_size", 20))
        self.generations = int(algorithm.get("generations", 10))
        self.seed = algorithm.get("seed")
        runner_config = config.get("runner") or {}
        self.max_retries = max(0, int(runner_config.get("max_retries", 2)))
        self.repeatability_tolerance = float(runner_config.get("repeatability_tolerance", 0.001))

        self.scorer: Optional[Scorer] = None          # set by dry_run()
        self.reference: Optional[RunMetrics] = None   # the baseline's metrics, set by dry_run()
        self.evaluations: list[Evaluation] = []       # every candidate, in order
        self.cache: dict[tuple, Evaluation] = {}      # gene key -> first evaluation of it
        self.evaluations_done = 0
        self.evaluations_total = self.population_size * self.generations
        self.generation = 0
        self.started = time.monotonic()
        self.loop_started: Optional[float] = None     # when optimize() began (for the ETA)

    # --- Scoring ----------------------------------------------------------------
    def _read_metrics(self, result: RunResult) -> tuple[Optional[RunMetrics], str]:
        """Measure a finished run from its log. Returns (metrics, problem):
        metrics is None if there's no usable log, and problem says why."""
        if result.error is not None or result.controller_dir is None:
            return None, result.error or "no log folder"
        try:
            log = read_controller_log(result.controller_dir / CONTROLLER_LOG_NAME)
            check = check_columns(log.columns, self.spec)
            if not check.ok:
                return None, check.describe()
            return compute_metrics(log, self.spec, self.metric_settings), ""
        except (LogError, KeyError, ValueError) as error:
            return None, str(error)

    def _evaluate(self, eval_id: str, generation: int, x: np.ndarray, payload: bytes, result: RunResult) -> Evaluation:
        """Score one finished run, and report it on the console."""
        assert self.scorer is not None
        metrics, problem = self._read_metrics(result)
        if result.error is None and not result.gains_match:
            status, metrics = "mismatch", None
        elif metrics is None:
            status = "failed"
        elif metrics.crashed:
            status = "crashed"
        elif self.scorer.ended_early(metrics):
            status = "ended_early"
        else:
            status = "ok"
        objectives = self.scorer.objectives(metrics)
        evaluation = Evaluation(eval_id, generation, np.asarray(x, dtype=float), status, metrics, objectives,
                                self.scorer.weighted(objectives), result.run_dir, payload)

        where = result.run_dir.name if result.run_dir else "no run folder"
        if status == "ok":
            reporting.info(f"{eval_id} OK ({where}, {result.duration:0.0f} s): pos {metrics.position:.4f} m, "
                           f"vel {metrics.velocity:.4f} m/s, effort {metrics.effort:.4f} -> score {evaluation.weighted:.4f}")
        elif status == "crashed":
            reporting.warning(f"{eval_id} CRASHED at {metrics.crash_time:0.2f} s ({metrics.crash_reason}) -> score {evaluation.weighted:.1f}")
        elif status == "ended_early":
            reporting.warning(f"{eval_id} LOG ENDED EARLY at {metrics.final_time:0.2f} s -> score {evaluation.weighted:.1f}")
        elif status == "mismatch":
            reporting.warning(f"{eval_id} LOGGED GAINS DON'T MATCH THE CANDIDATE ({where})")
        else:
            reporting.warning(f"{eval_id} FAILED: {problem}")
        if result.returncode not in (0, None) and status != "failed":
            reporting.warning(f"{eval_id} simulator exit code {result.returncode} (see {result.console_log.name})")
        return evaluation

    def _record(self, evaluation: Evaluation) -> None:
        """Keep an evaluation: in memory, in the cache, and in history.csv."""
        self.evaluations.append(evaluation)
        self.cache.setdefault(_gene_key(evaluation.x), evaluation)
        row = {
            "eval_id": evaluation.eval_id,
            "generation": evaluation.generation,
            "status": evaluation.status,
            "run_dir": evaluation.run_dir or "",
            "obj_position": evaluation.objectives[0],
            "obj_velocity": evaluation.objectives[1],
            "obj_effort": evaluation.objectives[2],
            "weighted": evaluation.weighted,
        }
        metrics = evaluation.metrics
        if metrics is not None:
            row.update(position_rms=metrics.position, velocity_rms=metrics.velocity, effort_rms=metrics.effort,
                       saturation_fraction=metrics.saturation_fraction, final_time=metrics.final_time,
                       crash_time="" if metrics.crash_time is None else metrics.crash_time, crash_reason=metrics.crash_reason)
        self.files.append_history(row, self.codec.physical_values(evaluation.x))

    # --- Progress ---------------------------------------------------------------
    def best(self) -> Optional[Evaluation]:
        """The completed candidate with the lowest weighted sum (the baseline
        dry runs included), or None if nothing completed."""
        completed = [e for e in self.evaluations if e.status == "ok"]
        return min(completed, key=lambda e: e.weighted) if completed else None

    def _eta(self) -> Optional[float]:
        """Seconds left, extrapolated from the average time per candidate so far."""
        if self.loop_started is None or not self.evaluations_done:
            return None
        per_evaluation = (time.monotonic() - self.loop_started) / self.evaluations_done
        return per_evaluation * max(0, self.evaluations_total - self.evaluations_done)

    def progress(self, state: str, **extra) -> None:
        """Rewrite progress.json (polled by the GUI). `state` is one of
        starting, dry_run, running, done, stopped, failed."""
        best = self.best()
        self.files.update_progress(
            state=state,
            platform=self.ctx.platform,
            controller=self.ctx.controller_name,
            algorithm=self.algorithm_name,
            generation=self.generation,
            generations=self.generations,
            evaluations_done=self.evaluations_done,
            evaluations_total=self.evaluations_total,
            crashes=sum(e.status in ("crashed", "ended_early") for e in self.evaluations),
            failures=sum(e.status in ("failed", "mismatch") for e in self.evaluations),
            best=best.summary() if best else None,
            baseline=self.reference.to_dict() if self.reference else None,
            elapsed_seconds=time.monotonic() - self.started,
            eta_seconds=self._eta(),
            **extra,
        )

    # --- Step 3: the dry run ----------------------------------------------------
    def dry_run(self) -> None:
        """Fly the untouched baseline twice. Anything wrong with the setup
        (log columns, log folder naming, the baseline itself crashing) ends
        the session here, before a single candidate is written."""
        reporting.info("BASELINE DRY RUN x2: checking columns, log folder and repeatability")
        self.progress("dry_run")
        payload = self.store.original_bytes
        requests = [RunRequest(f"baseline_{n}", f"BASELINE DRY RUN {n}", payload) for n in (1, 2)]
        results = self.runner.run_batch(requests)

        metrics: list[RunMetrics] = []
        for request in requests:
            result = results[request.key]
            if result.error is not None or result.controller_dir is None:
                raise SessionError(f"{request.label} failed: {result.error or 'no log folder'} (see {result.console_log})")
            # The GA finds a controller's files by the upper-case naming
            # convention (context.gains_path_for); check the simulator agrees.
            if result.controller_dir.name != self.ctx.controller_name:
                raise SessionError(f"{request.label} logged to '{result.controller_dir.name}', expected '{self.ctx.controller_name}': "
                                   "the controller's folders don't follow the <CONTROLLER_KEY in upper case> convention")
            if not result.gains_match:
                raise SessionError(f"{request.label}: the gains logged in {result.run_dir} differ from {self.ctx.gains_path}")
            try:
                log = read_controller_log(result.controller_dir / CONTROLLER_LOG_NAME)
            except LogError as error:
                raise SessionError(f"{request.label}: {error}") from error
            check = check_columns(log.columns, self.spec)
            if not check.ok:
                raise SessionError(f"{self.ctx.label} is not GA-ready: {check.describe()}")
            run = compute_metrics(log, self.spec, self.metric_settings)
            if run.crashed:
                raise SessionError(f"{request.label} crashed at {run.crash_time:0.2f} s ({run.crash_reason}); "
                                   "the baseline must fly the whole trajectory")
            metrics.append(run)
            reporting.info(f"{request.label} OK ({result.run_dir.name}): {check.describe()}, pos {run.position:.5f} m, "
                           f"vel {run.velocity:.5f} m/s, effort {run.effort:.5f}, {run.final_time:0.2f} s flown")

        # Largest relative difference between the two runs over the three
        # objectives. Non-zero means the simulation isn't deterministic, so
        # every score carries that much noise.
        difference = float(np.max(np.abs(metrics[0].raw() - metrics[1].raw()) / np.maximum(np.abs(metrics[0].raw()), 1e-12)))
        if difference > self.repeatability_tolerance:
            reporting.warning(f"BASELINE RUNS DIFFER BY UP TO {100 * difference:0.3f}% (tolerance {100 * self.repeatability_tolerance:0.3f}%): "
                              "something in the simulation isn't deterministic, so scores will be noisy")
        else:
            reporting.info(f"BASELINE IS REPEATABLE (max difference {100 * difference:0.4f}%)")

        self.reference = metrics[0]
        self.scorer = Scorer(self.reference, self.config)
        # Both dry runs go into the history. The first is also cached under
        # the baseline genes, so when the optimizer's first generation
        # contains the baseline (it always does), it isn't flown a third time.
        for request, run in zip(requests, metrics):
            objectives = self.scorer.objectives(run)
            self._record(Evaluation(request.key, 0, self.codec.x0, "ok", run, objectives, self.scorer.weighted(objectives),
                                    results[request.key].run_dir, payload))

    # --- Step 4: optimization ---------------------------------------------------
    def _run_generation(self, X: np.ndarray) -> list[Evaluation]:
        """Simulate one generation and return its evaluations, row for row.

        Candidates already simulated (the seeded baseline, or a vector the
        optimizer hands out again) come from the cache instead of a new run.
        A candidate whose logged gains don't match is re-run, up to
        max_retries times.
        """
        generation, done_before = self.generation, self.evaluations_done

        # The distinct, not-yet-simulated candidates: gene key -> (eval_id, genes, gains bytes).
        todo: dict[tuple, tuple[str, np.ndarray, bytes]] = {}
        for index, x in enumerate(X):
            key = _gene_key(x)
            if key not in self.cache and key not in todo:
                todo[key] = (f"g{generation:03d}_c{index + 1:03d}", x, dumps(self.codec.decode(x)))

        for attempt in range(self.max_retries + 1):
            if not todo:
                break
            # Run keys must be unique per batch; retries get a suffix so their
            # console logs don't overwrite the first attempt's.
            keys = {(eval_id if attempt == 0 else f"{eval_id}_retry{attempt}"): key for key, (eval_id, _x, _p) in todo.items()}

            def on_result(result: RunResult) -> None:
                eval_id, x, payload = todo[keys[result.key]]
                evaluation = self._evaluate(eval_id, generation, x, payload, result)
                if evaluation.status == "mismatch" and attempt < self.max_retries:
                    reporting.warning(f"{eval_id} RE-QUEUED (attempt {attempt + 2} of {self.max_retries + 1})")
                    return   # not recorded: it stays in `todo` for the next attempt
                self._record(evaluation)
                self.evaluations_done += 1
                self.progress("running")

            requests = [RunRequest(run_key, f"GEN {generation}/{self.generations} {todo[key][0]}", todo[key][2]) for run_key, key in keys.items()]
            self.runner.run_batch(requests, on_result)
            # Whatever was recorded is now in the cache; the rest is retried.
            todo = {key: item for key, item in todo.items() if key not in self.cache}

        # Cached and repeated candidates count toward the total too.
        self.evaluations_done = done_before + len(X)
        return [self.cache[_gene_key(x)] for x in X]

    def optimize(self) -> None:
        """The generation loop: ask, simulate, tell."""
        assert self.scorer is not None
        optimizer = Optimizer(self.algorithm_name, self.population_size, self.generations,
                              self.codec.xl, self.codec.xu, self.codec.x0, len(OBJECTIVES), self.seed)
        reporting.info(f"OPTIMIZING {self.codec.n_var} GENE(S) WITH {ALGORITHMS[self.algorithm_name]}: "
                       f"{self.population_size} candidates x {self.generations} generations, "
                       f"{self.runner.settings.concurrent_runs} concurrent run(s)")
        self.loop_started = time.monotonic()
        while optimizer.has_next():
            self.generation = optimizer.generation
            X = optimizer.ask()
            self.progress("running")
            reporting.status(f"GENERATION {self.generation}/{self.generations}: {len(X)} candidates")
            evaluations = self._run_generation(X)
            optimizer.tell(np.array([e.objectives for e in evaluations]), np.array([e.weighted for e in evaluations]))

            self._report_generation(evaluations)
            # Written every generation, so a stopped session still has results.
            self.write_pareto()

    def _report_generation(self, evaluations: list[Evaluation]) -> None:
        """One summary line per generation."""
        assert self.scorer is not None
        completed = [e.weighted for e in evaluations if e.status == "ok"]
        crashed = sum(e.status in ("crashed", "ended_early") for e in evaluations)
        failed = sum(e.status in ("failed", "mismatch") for e in evaluations)
        this_generation = f"best {min(completed):.4f}, median {float(np.median(completed)):.4f}" if completed else "nothing completed"
        best = self.best()
        overall = f"{best.weighted:.4f} ({best.eval_id})" if best else "--"
        reporting.status(
            f"GENERATION {self.generation}/{self.generations} DONE: {this_generation}, {crashed} crashed, {failed} failed "
            f"| overall best {overall} | baseline {self.scorer.baseline_score:.4f} | ETA {format_duration(self._eta())}"
        )

    # --- Step 6: results --------------------------------------------------------
    def write_pareto(self, tuned_file: Optional[Path] = None) -> list[Evaluation]:
        """Write pareto.json, and each front member's gains under pareto/,
        from every completed candidate so far. Returns the front, best
        (lowest weighted sum) first."""
        # One evaluation per distinct gene vector (the earliest one wins:
        # iterating newest-first lets older entries overwrite newer ones).
        unique = {_gene_key(e.x): e for e in reversed(self.evaluations) if e.status == "ok"}
        front = sorted(non_dominated(list(unique.values())), key=lambda e: e.weighted)
        solutions = []
        for evaluation in front:
            gains_file = self.files.pareto_dir / f"gains_{self.ctx.controller_name}_{evaluation.eval_id}.json"
            if not gains_file.exists():
                self.files.pareto_dir.mkdir(parents=True, exist_ok=True)
                gains_file.write_bytes(evaluation.payload)
            solutions.append({**evaluation.summary(), "gains_file": str(gains_file)})
        best = self.best()
        write_json(self.files.pareto_path, {
            "platform": self.ctx.platform,
            "controller": self.ctx.controller_name,
            "objectives": list(OBJECTIVES),
            "weights": dict(zip(OBJECTIVES, self.scorer.weights.tolist())) if self.scorer else None,
            "best": best.eval_id if best else None,
            "tuned_file": str(tuned_file) if tuned_file else None,
            "solutions": solutions,
        })
        return front

    def finalize(self, state: str) -> None:
        """Save the best candidate to ga_tuned/ (unless the baseline is still
        the best) and write the final pareto.json and progress.json."""
        best = self.best()
        tuned_file = None
        if best is None:
            reporting.warning("NO COMPLETED CANDIDATE -- NOTHING TO SAVE")
        elif best.generation == 0:
            reporting.warning("NO CANDIDATE BEAT THE BASELINE -- nothing written to ga_tuned/")
        else:
            tuned_file = self.store.save_tuned(best.payload)
            reporting.info(f"BEST CANDIDATE {best.eval_id} (score {best.weighted:.4f}, baseline {self.scorer.baseline_score:.4f})")
            reporting.info(f"TUNED GAINS SAVED: {tuned_file}")
        front = self.write_pareto(tuned_file)
        if is_multi_objective(self.algorithm_name) and front:
            reporting.info(f"PARETO FRONT: {len(front)} solution(s), gains in {self.files.pareto_dir}")
        self.progress(state, tuned_file=str(tuned_file) if tuned_file else None)


# =============================================================================
# The whole session
# =============================================================================
def _raise_keyboard_interrupt(_signum, _frame) -> None:
    raise KeyboardInterrupt


def run_session(config: dict, ctx: SimContext) -> int:
    """Run one session (steps 1-6 in the file header) and return its exit
    code. Pre-flight must already have passed."""
    # --- Step 1: setup. Nothing has been touched yet, so a problem here is EXIT_NOT_STARTED.
    try:
        recovered = recover_stale_lock()
        if recovered:
            reporting.warning(recovered)
        default_factor = float(config.get("default_bound_factor", 10.0))
        selections = [Selection.from_config(item, default_factor) for item in config.get("tune") or []]
        codec = GainsCodec(json.loads(ctx.gains_path.read_bytes()), selections, config.get("exclude_patterns") or [])
        runner_settings = RunnerSettings.from_config(config)
    except (StoreError, ValueError, OSError) as error:
        reporting.error(f"INVALID GA SETUP: {error}")
        return EXIT_NOT_STARTED

    timestamp = time.strftime("%Y_%m_%d_%H_%M_%S")
    files = SessionFiles(ctx.ga_log_dir / timestamp, codec.names)
    (files.session_dir / "session.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    reporting.info(f"SESSION {timestamp}: {ctx.label}, {codec.n_var} gene(s), logs in {files.session_dir}")

    store = GainsStore(ctx, timestamp)
    runner = SimRunner(ctx, runner_settings, files.console_dir, store.write_candidate)
    session = GaSession(config, ctx, codec, store, files, runner)

    # Ctrl+C / the GUI's Stop (SIGINT) and SIGTERM both raise
    # KeyboardInterrupt, which ends the session cleanly below. SIGINT is
    # installed explicitly: a process started in the background inherits it
    # as "ignored", which would make the GUI's Stop do nothing.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    # --- Steps 2-4: snapshot, dry run, optimize.
    state, exit_code = "done", EXIT_DONE
    try:
        store.begin()
        reporting.info(f"BASELINE SNAPSHOT: {store.snapshot_path}")
        session.dry_run()
        session.optimize()
    except KeyboardInterrupt:
        reporting.warning("GA INTERRUPTED -- STOPPING ACTIVE RUNS")
        state, exit_code = "stopped", EXIT_STOPPED
    except (SessionError, StoreError) as error:
        reporting.error(str(error))
        state, exit_code = "failed", EXIT_FAILED
    except Exception as error:  # a bug: still restore the gains below
        reporting.error(f"UNEXPECTED ERROR: {error!r}")
        traceback.print_exc()
        state, exit_code = "failed", EXIT_FAILED
    finally:
        # --- Step 5: clean up. Further Ctrl+C / SIGTERM are ignored from
        # here on: being interrupted now is what could leave a candidate's
        # gains in gains_<CTRL>.json.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        runner.terminate_all()
        if store.finish():
            reporting.info(f"ORIGINAL GAINS RESTORED: {ctx.gains_path}")
        deactivate_wrapper_session_marker(ctx)

    # --- Step 6: results. Only possible if the dry run got as far as a scorer.
    try:
        if session.scorer is not None:
            session.finalize(state)
        else:
            session.progress(state)
    except Exception as error:
        reporting.error(f"COULD NOT WRITE RESULTS: {error!r}")
        exit_code = exit_code or EXIT_FAILED
    reporting.info(f"GA SESSION {state.upper()} after {format_duration(time.monotonic() - session.started)}")
    return exit_code
