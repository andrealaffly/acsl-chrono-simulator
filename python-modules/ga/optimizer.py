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
# File:        optimizer.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     A thin wrapper around pymoo's "ask and tell" interface. The usual way
#     to use pymoo is to hand it an objective function and let it call that
#     function itself; here every evaluation is a whole simulator run, and
#     the session wants to run many of them in parallel processes. So the
#     loop is turned inside out:
#
#         while optimizer.has_next():
#             X = optimizer.ask()          # pymoo proposes a generation
#             F = <simulate every row of X>
#             optimizer.tell(F)            # pymoo learns from the scores
#
#     Three algorithms are offered:
#       nsga2 - NSGA-II, multi-objective: keeps a Pareto front over the
#               three objectives (position, velocity, effort)
#       de    - differential evolution (DE/rand/1/bin) on the weighted sum
#       ga    - a genetic algorithm (SBX crossover, polynomial mutation) on
#               the weighted sum
#
#     The initial population is random within the bounds, except its first
#     member, which is always the baseline gains -- so the search starts
#     from at least one candidate known to fly.
#
#     pymoo is only imported inside Optimizer: the GUI imports this module
#     for ALGORITHMS, and must keep working (and say "install pymoo")
#     without it.
###############################################################################

from __future__ import annotations

from typing import Optional

import numpy as np

# Algorithm key (as written in ga-config.yaml) -> label shown to the user.
ALGORITHMS = {
    "nsga2": "NSGA-II (Pareto front)",
    "de": "Differential evolution (weighted sum)",
    "ga": "Genetic algorithm (weighted sum)",
}


def is_multi_objective(name: str) -> bool:
    """True if the algorithm optimizes the objective vector, not the weighted sum."""
    return name == "nsga2"


class Optimizer:
    def __init__(self, name: str, population_size: int, generations: int, xl: np.ndarray, xu: np.ndarray, x0: np.ndarray,
                 n_obj: int, seed: Optional[int]) -> None:
        """
        name             one of ALGORITHMS
        population_size  candidates per generation (at least 4)
        generations      generations to run, the initial population included
        xl, xu, x0       gene bounds and the baseline genes (from GainsCodec)
        n_obj            number of objectives (used by nsga2 only)
        seed             random seed, or None for a random session
        """
        from pymoo.core.evaluator import Evaluator
        from pymoo.core.problem import Problem
        from pymoo.termination import get_termination

        if name not in ALGORITHMS:
            raise ValueError(f"unknown algorithm '{name}' (choose from {', '.join(ALGORITHMS)})")
        if population_size < 4 or generations < 1:
            raise ValueError("population_size must be at least 4 and generations at least 1")

        self.name = name
        self.generation = 1      # number of the generation the next ask() returns
        self._evaluator = Evaluator()
        self._pending = None     # the population handed out by ask(), awaiting tell()
        # The problem only describes the search space; it is never asked to
        # evaluate anything itself (the scores arrive through tell()).
        self.problem = Problem(n_var=len(x0), n_obj=n_obj if is_multi_objective(name) else 1, xl=xl, xu=xu)

        # Random initial population, with the baseline as its first member.
        initial = np.random.default_rng(seed).uniform(xl, xu, size=(population_size, len(x0)))
        initial[0] = x0
        self.algorithm = self._make_algorithm(name, population_size, initial)
        self.algorithm.setup(self.problem, termination=get_termination("n_gen", generations), seed=seed, verbose=False)

    @staticmethod
    def _make_algorithm(name: str, population_size: int, initial: np.ndarray):
        if name == "nsga2":
            from pymoo.algorithms.moo.nsga2 import NSGA2
            return NSGA2(pop_size=population_size, sampling=initial)
        if name == "de":
            from pymoo.algorithms.soo.nonconvex.de import DE
            return DE(pop_size=population_size, sampling=initial, variant="DE/rand/1/bin", CR=0.9, F=0.8)
        from pymoo.algorithms.soo.nonconvex.ga import GA
        return GA(pop_size=population_size, sampling=initial)

    def has_next(self) -> bool:
        """False once every generation has been told its scores."""
        return bool(self.algorithm.has_next())

    def ask(self) -> np.ndarray:
        """The next generation's gene vectors, shape (candidates, genes)."""
        self._pending = self.algorithm.ask()
        return np.asarray(self._pending.get("X"), dtype=float)

    def tell(self, objectives: np.ndarray, weighted: np.ndarray) -> None:
        """Hand back the scores of the last ask(), row for row.

        objectives  (candidates, 3) normalized objectives -- used by nsga2
        weighted    (candidates,)   weighted sums         -- used by de/ga
        """
        from pymoo.problems.static import StaticProblem

        scores = objectives if is_multi_objective(self.name) else np.asarray(weighted, dtype=float)[:, None]
        # StaticProblem "evaluates" by returning the scores given to it,
        # which is how precomputed results are attached to a population.
        self._evaluator.eval(StaticProblem(self.problem, F=scores), self._pending)
        self.algorithm.tell(infills=self._pending)
        self._pending = None
        self.generation += 1
