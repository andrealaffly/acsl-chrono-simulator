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
# File:        gains_codec.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Converts between a controller's gains JSON and the optimizer's flat
#     vector of genes (one float per tuned number).
#
#     DISCOVERY
#     Tunable entries are found by walking the JSON, not from a list kept per
#     controller, so a new controller is tunable without code changes:
#       - every {"scaling_coef": ..., "matrix": ...} object is a MATRIX entry
#       - every plain number is a SCALAR entry
#     Each is addressed by its dotted path, e.g. "TRANSLATIONAL.KP_translational".
#
#     MODES (how an entry is turned into genes)
#       scaling  - one gene: the scaling_coef                    (any matrix)
#       diagonal - one gene per nonzero diagonal element          (square)
#       full     - one gene per element of the Cholesky factor    (symmetric
#                  L (lower triangle, n(n+1)/2 genes); the         positive-
#                  matrix is rebuilt as L L^T                      definite)
#       elements - one gene per nonzero element                   (any matrix)
#       value    - one gene: the number itself                    (scalars)
#
#     WHY LOG SPACE
#     A gene is log(|value|), and the value's sign is kept from the baseline:
#         value = sign * exp(gene)
#     so the search range "value / k  ..  value * k" becomes the symmetric
#     interval "gene0 - log k  ..  gene0 + log k". This also means a gain
#     can never change sign or become exactly zero, which for most gains
#     would make the controller meaningless.
#
#     In "full" mode the Cholesky diagonal is in log space (so it stays
#     positive, which keeps L L^T positive-definite) and the off-diagonal
#     terms are plain values, as in the pychrono GA tuner.
###############################################################################

from __future__ import annotations

import copy
import fnmatch
import json
import math
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from .utils import get_dotted, set_dotted

# Every mode, with the description shown by --list and in the GUI.
MODE_DESCRIPTIONS = {
    "scaling": "scaling coefficient",
    "diagonal": "nonzero diagonal elements",
    "full": "full SPD matrix (Cholesky)",
    "elements": "every nonzero element",
    "value": "value",
}


def _is_number(value: Any) -> bool:
    # bool is a subclass of int in Python, but a true/false switch is not a gain.
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _as_matrix(value: Any) -> np.ndarray:
    """A JSON matrix as a 2-D array; a flat list becomes a column."""
    matrix = np.array(value, dtype=float)
    return matrix[:, None] if matrix.ndim == 1 else matrix


# =============================================================================
# Discovery
# =============================================================================
@dataclass
class Entry:
    """One tunable-looking entry of a gains file."""

    path: str                # dotted path in the JSON
    kind: str                # "matrix" | "scalar"
    shape: tuple[int, ...]   # matrix shape; () for a scalar
    modes: list[str]         # the modes this entry allows (empty = can't be tuned)
    excluded: bool           # matches one of ga-config.yaml's exclude_patterns
    note: str = ""           # why it can't be tuned, if modes is empty

    @property
    def summary(self) -> str:
        return f"{self.shape[0]}x{self.shape[1]} matrix" if self.kind == "matrix" else "scalar"


def _matrix_modes(entry: dict) -> tuple[list[str], tuple[int, ...]]:
    """The modes a {"scaling_coef", "matrix"} entry allows, judged from its
    values, plus its shape. ([], ()) if it isn't a usable numeric matrix."""
    try:
        matrix = _as_matrix(entry["matrix"])
    except (TypeError, ValueError):
        return [], ()
    if matrix.ndim != 2 or not matrix.size or not _is_number(entry["scaling_coef"]):
        return [], ()
    square = matrix.shape[0] == matrix.shape[1]
    modes = []
    if entry["scaling_coef"] > 0:
        modes.append("scaling")
    if square and np.any(np.diag(matrix)):
        modes.append("diagonal")
    # "full" needs a Cholesky factor, which exists only for a symmetric
    # positive-definite matrix (all eigenvalues > 0).
    if square and np.allclose(matrix, matrix.T) and np.all(np.linalg.eigvalsh(0.5 * (matrix + matrix.T)) > 0):
        modes.append("full")
    if np.any(matrix):
        modes.append("elements")
    return modes, matrix.shape


def discover_entries(gains: dict, exclude_patterns: list[str]) -> list[Entry]:
    """Every tunable-looking entry of a gains JSON, in file order.

    `exclude_patterns` are case-insensitive globs on the dotted path (e.g.
    "*filter*"); matching entries are still listed, marked excluded.
    """
    patterns = [pattern.lower() for pattern in exclude_patterns or []]
    entries: list[Entry] = []

    def walk(node: dict, prefix: str) -> None:
        for key, value in node.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            excluded = any(fnmatch.fnmatchcase(path.lower(), pattern) for pattern in patterns)
            if isinstance(value, dict) and "scaling_coef" in value and "matrix" in value:
                modes, shape = _matrix_modes(value)
                entries.append(Entry(path, "matrix", shape, modes, excluded, "" if modes else "not a usable matrix"))
            elif _is_number(value):
                # Zero can't be scaled multiplicatively (log 0 is undefined).
                entries.append(Entry(path, "scalar", (), ["value"] if value else [], excluded, "" if value else "zero (can't scale)"))
            elif isinstance(value, dict):
                walk(value, path)   # a plain section: look inside it

    walk(gains, "")
    return entries


# =============================================================================
# Selections and blocks of genes
# =============================================================================
@dataclass
class Selection:
    """One "tune" item: which entry, in which mode, over which range."""

    path: str
    mode: str
    bound_factor: float   # each gene may move between value / k and value * k

    @classmethod
    def from_config(cls, item: Any, default_bound_factor: float) -> "Selection":
        """Parse one item of ga-config.yaml's "tune" list."""
        if not isinstance(item, dict) or "path" not in item or "mode" not in item:
            raise ValueError(f"tune entries need 'path' and 'mode', got {item!r}")
        factor = float(item.get("bound_factor", default_bound_factor))
        if factor <= 1.0:
            raise ValueError(f"{item['path']}: bound_factor must be greater than 1")
        return cls(str(item["path"]), str(item["mode"]), factor)


@dataclass
class _Block:
    """The genes of one selection, and how to use them."""

    names: list[str]                              # one name per gene (history.csv columns)
    x0: list[float]                               # gene values that reproduce the baseline
    lower: list[float]                            # lower bound per gene
    upper: list[float]                            # upper bound per gene
    apply: Callable[[dict, np.ndarray], None]     # write the genes into a gains dict
    physical: Callable[[np.ndarray], list[float]] # the genes in physical units


def _log_genes(values: list[float], span: float) -> tuple[list[float], list[float], list[float], list[float]]:
    """Log-space genes around `values`: (signs, x0, lower, upper), where
    each gene may move `span` = log(bound_factor) either way."""
    signs = [math.copysign(1.0, value) for value in values]
    x0 = [math.log(abs(value)) for value in values]
    return signs, x0, [x - span for x in x0], [x + span for x in x0]


def _build_block(baseline: dict, selection: Selection) -> _Block:
    """The genes for one selection (see the MODES in the file header)."""
    path, mode = selection.path, selection.mode
    reference = get_dotted(baseline, path)
    span = math.log(selection.bound_factor)

    # --- value: a plain number --------------------------------------------
    if mode == "value":
        signs, x0, lower, upper = _log_genes([float(reference)], span)

        def value_of(genes: np.ndarray) -> float:
            value = signs[0] * math.exp(genes[0])
            if not isinstance(reference, int):
                return value
            # Integer entries (horizons, iteration counts) stay nonzero
            # integers: round, and never round down to 0.
            return int(round(value)) or int(signs[0])

        return _Block([path], x0, lower, upper,
                      lambda data, g: set_dotted(data, path, value_of(g)),
                      lambda g: [float(value_of(g))])

    # --- scaling: the matrix's scaling_coef --------------------------------
    if mode == "scaling":
        # scaling_coef is always positive here (_matrix_modes requires it).
        signs, x0, lower, upper = _log_genes([float(reference["scaling_coef"])], span)
        return _Block([f"{path}.scaling_coef"], x0, lower, upper,
                      lambda data, g: get_dotted(data, path).update(scaling_coef=math.exp(g[0])),
                      lambda g: [math.exp(g[0])])

    # --- the matrix modes ---------------------------------------------------
    matrix = _as_matrix(reference["matrix"])
    # A flat list in the JSON (a column vector) is written back flat.
    flat = not isinstance(reference["matrix"][0], list)

    def write(data: dict, values: np.ndarray) -> None:
        get_dotted(data, path)["matrix"] = values[:, 0].tolist() if flat else values.tolist()

    # diagonal / elements: one log-space gene per chosen nonzero cell; every
    # other cell keeps its baseline value.
    if mode in ("diagonal", "elements"):
        rows, cols = matrix.shape
        cells = [(i, j) for i in range(rows) for j in range(cols) if matrix[i, j] and (mode == "elements" or i == j)]
        signs, x0, lower, upper = _log_genes([matrix[cell] for cell in cells], span)

        def cell_values(genes: np.ndarray) -> list[float]:
            return [sign * math.exp(gene) for sign, gene in zip(signs, genes)]

        def apply_cells(data: dict, genes: np.ndarray) -> None:
            values = matrix.copy()
            for cell, value in zip(cells, cell_values(genes)):
                values[cell] = value
            write(data, values)

        return _Block([f"{path}[{i},{j}]" for i, j in cells], x0, lower, upper, apply_cells, cell_values)

    # full: the lower-triangular Cholesky factor L of the matrix (M = L L^T),
    # with a log-space diagonal. Any genes give a positive diagonal, so the
    # rebuilt M is always symmetric positive-definite.
    size = matrix.shape[0]
    factor = np.linalg.cholesky(0.5 * (matrix + matrix.T))
    cells = [(i, j) for i in range(size) for j in range(i + 1)]   # lower triangle, row by row
    names, x0, lower, upper = [], [], [], []
    for i, j in cells:
        names.append(f"{path}.L[{i},{j}]")
        if i == j:
            value, half_range = math.log(factor[i, i]), span
        else:
            # Off-diagonal terms often start at zero, so their range can't be
            # a multiple of their own value. It is set by the diagonal terms
            # they couple instead: sqrt(L_ii * L_jj) * (1 - 1/k).
            value = float(factor[i, j])
            half_range = math.sqrt(factor[i, i] * factor[j, j]) * (1.0 - 1.0 / selection.bound_factor)
        x0.append(value)
        lower.append(value - half_range)
        upper.append(value + half_range)

    def factor_values(genes: np.ndarray) -> list[float]:
        return [math.exp(gene) if i == j else float(gene) for (i, j), gene in zip(cells, genes)]

    def apply_full(data: dict, genes: np.ndarray) -> None:
        lower_factor = np.zeros((size, size))
        for cell, value in zip(cells, factor_values(genes)):
            lower_factor[cell] = value
        values = lower_factor @ lower_factor.T
        write(data, 0.5 * (values + values.T))   # symmetrize away rounding noise

    return _Block(names, x0, lower, upper, apply_full, factor_values)


# =============================================================================
# GainsCodec
# =============================================================================
class GainsCodec:
    """Gene vector <-> gains dict, for a fixed baseline and selection.

    The gene vector is the blocks of every selection, concatenated in the
    order the selections were given.
    """

    def __init__(self, baseline: dict, selections: list[Selection], exclude_patterns: list[str]) -> None:
        if not selections:
            raise ValueError("nothing selected to tune (the 'tune' list is empty)")

        # Check every selection against what discovery found in this file.
        entries = {entry.path: entry for entry in discover_entries(baseline, exclude_patterns)}
        paths = [selection.path for selection in selections]
        for selection in selections:
            entry = entries.get(selection.path)
            if entry is None:
                raise ValueError(f"'{selection.path}' is not a tunable entry of this gains file")
            if entry.excluded:
                raise ValueError(f"'{selection.path}' matches exclude_patterns and can't be tuned")
            if selection.mode not in entry.modes:
                raise ValueError(f"'{selection.path}' can't use mode '{selection.mode}' (allowed: {', '.join(entry.modes) or 'none'})")
            if paths.count(selection.path) > 1:
                raise ValueError(f"'{selection.path}' is selected more than once")

        self.baseline = baseline
        self._blocks = [_build_block(baseline, selection) for selection in selections]
        self.names = [name for block in self._blocks for name in block.names]
        self.x0 = np.array([value for block in self._blocks for value in block.x0])      # the baseline
        self.xl = np.array([value for block in self._blocks for value in block.lower])   # lower bounds
        self.xu = np.array([value for block in self._blocks for value in block.upper])   # upper bounds

    @property
    def n_var(self) -> int:
        """Number of genes."""
        return len(self.names)

    def _split(self, x: np.ndarray) -> list[np.ndarray]:
        """Cut a gene vector into one piece per block."""
        x = np.asarray(x, dtype=float)
        if x.shape != (self.n_var,):
            raise ValueError(f"expected {self.n_var} genes, got shape {x.shape}")
        offsets = np.cumsum([0] + [len(block.names) for block in self._blocks])
        return [x[start:end] for start, end in zip(offsets[:-1], offsets[1:])]

    def decode(self, x: np.ndarray) -> dict:
        """The full gains dict for a gene vector (untuned entries as in the baseline)."""
        data = copy.deepcopy(self.baseline)
        for block, genes in zip(self._blocks, self._split(x)):
            block.apply(data, genes)
        return data

    def physical_values(self, x: np.ndarray) -> list[float]:
        """The genes in physical units -- what they mean in the JSON, or the
        Cholesky factor's elements in "full" mode -- for history.csv."""
        return [value for block, genes in zip(self._blocks, self._split(x)) for value in block.physical(genes)]


def dumps(data: dict) -> bytes:
    """The exact bytes written for a candidate. Each run copies the gains it
    read into its log folder, and that copy is compared byte for byte with
    these, so this formatting must stay deterministic."""
    return (json.dumps(data, indent=2) + "\n").encode("utf-8")
