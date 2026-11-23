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
# File:        gains_view.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     GainsEditor: the Settings page's "Gains" tab. Loads the gains file of
#     the controller selected on the Simulator tab,
#         chrono-assets/parameters/<platform>/<CONTROLLER>/gains_<CONTROLLER>.json
#     (the file every run -- single or wrapper -- reads), lets every number
#     in it be edited in place, and writes it back on Save.
#
#     Each {"scaling_coef", "matrix"} entry is a card with its scaling
#     coefficient and its matrix, exactly as written in the JSON; the
#     controller multiplies the two when it loads them. Cards flow in as many
#     columns as the window width allows. Big matrices stay readable by
#     their structure:
#       c * identity   ->  one field, "c · I₃₀" (editing it sets the whole
#                          diagonal)
#       diagonal       ->  just the diagonal, wrapped every DIAGONAL_PER_LINE
#       anything else  ->  the full grid with row/column numbers, scrolling
#                          past MAX_VISIBLE rows/columns
#     and "Show full matrix" opens the full grid of any card. The plain
#     numbers and switches of a section are collected into one card.
###############################################################################

from __future__ import annotations

import json
import math
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Any, Callable, Optional

from . import dialogs, theme
from .widgets import Chip, ScrollableFrame

MONO = ("DejaVu Sans Mono", 10)
CARD_WIDTH = 380         # target width of one column of cards
MAX_VISIBLE = 12         # rows/columns a grid shows before it scrolls
DIAGONAL_PER_LINE = 10   # values per line of a diagonal
_SUBSCRIPTS = str.maketrans("0123456789", "₀₁₂₃₄₅₆₇₈₉")


# The gains file a controller reads -- the simulator's own convention: the
# folders are named after the sim-config controller key in upper case.
def gains_path(repo_dir: Path, platform: str, controller_key: str) -> Path:
    name = controller_key.upper()
    return repo_dir / "chrono-assets" / "parameters" / platform / name / f"gains_{name}.json"


# --- Numbers ----------------------------------------------------------------------
def number_text(value: Any) -> str:
    return repr(value) if isinstance(value, float) else str(value)


# Parses an edited field, keeping the original's int/float type when the
# text allows it (so "2" typed over 2.0 is saved as 2.0). None if invalid.
def parse_number(text: str, like: Any) -> Optional[float]:
    try:
        value = float(text.strip())
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    return int(value) if isinstance(like, int) and value.is_integer() else value


# The JSON written on Save: matrix rows (and any other list of numbers) on
# one line, everything else two-space indented -- close to how the gains
# files are written by hand.
def to_json(value: Any, depth: int = 0) -> str:
    pad, inner = "  " * depth, "  " * (depth + 1)
    if isinstance(value, dict):
        items = [f"{inner}{json.dumps(key)}: {to_json(item, depth + 1)}" for key, item in value.items()]
        return "{\n" + ",\n".join(items) + f"\n{pad}}}" if items else "{}"
    if isinstance(value, list) and any(isinstance(item, (list, dict)) for item in value):
        return "[\n" + ",\n".join(f"{inner}{to_json(item, depth + 1)}" for item in value) + f"\n{pad}]"
    if isinstance(value, list):
        return "[" + ", ".join(to_json(item) for item in value) + "]"
    return json.dumps(value)


# A matrix entry the editor can show: a non-empty, rectangular list of rows
# (or a flat list, i.e. one column) of numbers.
def is_matrix_entry(value: Any) -> bool:
    if not (isinstance(value, dict) and "scaling_coef" in value and isinstance(value.get("matrix"), list) and value["matrix"]):
        return False
    rows = [row if isinstance(row, list) else [row] for row in value["matrix"]]
    return all(len(row) == len(rows[0]) and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in row) for row in rows)


# "identity" (c * I), "diagonal", or "matrix" (anything else, vectors too).
def matrix_structure(rows: list[list[Any]]) -> str:
    size = len(rows)
    if size < 2 or any(len(row) != size for row in rows):
        return "matrix"
    if any(rows[i][j] != 0 for i in range(size) for j in range(size) if i != j):
        return "matrix"
    diagonal = [rows[i][i] for i in range(size)]
    return "identity" if diagonal[0] != 0 and all(value == diagonal[0] for value in diagonal) else "diagonal"


class _NumberField(tk.Entry):
    """An editable number. Calls on_value(value) with every valid edit and
    turns red while the text isn't a number.

    At rest it blends into its card (plain grey, no box) so a card reads as
    a matrix rather than a form; hovering outlines it, and while it's being
    edited it becomes a dark field with a red outline."""

    def __init__(self, parent: tk.Widget, editor: "GainsEditor", value: Any, on_value: Callable[[Any], None], width: int, bg: str) -> None:
        self._var = tk.StringVar(value=number_text(value))
        super().__init__(parent, textvariable=self._var, width=width, justify="right", font=MONO, relief="flat", bg=bg,
                         fg=theme.DIM if value == 0 else theme.TEXT, insertbackground=theme.TEXT, highlightthickness=1,
                         highlightbackground=bg, highlightcolor=theme.RED, borderwidth=0, cursor="xterm")
        self._editor, self._like, self._on_value, self._bg = editor, value, on_value, bg
        self._var.trace_add("write", lambda *_a: self._changed())
        self.bind("<FocusIn>", lambda _e: self.configure(bg=theme.FIELD))
        self.bind("<FocusOut>", lambda _e: self.configure(bg=self._bg, highlightbackground=self._bg))
        self.bind("<Enter>", lambda _e: self.configure(highlightbackground=theme.BORDER))
        self.bind("<Leave>", lambda _e: self.configure(highlightbackground=self._bg))

    def _changed(self) -> None:
        value = parse_number(self._var.get(), self._like)
        self._editor.mark_invalid(self, value is None)
        if value is None:
            self.configure(fg=theme.RED)
            return
        self.configure(fg=theme.DIM if value == 0 else theme.TEXT)
        self._on_value(value)
        self._editor.mark_dirty()


def _field_width(values: list[Any]) -> int:
    return min(14, max(5, max(len(number_text(value)) for value in values) + 1))


# --- Cards --------------------------------------------------------------------------
class _Card(tk.Frame):
    def __init__(self, parent: tk.Widget, title: str) -> None:
        super().__init__(parent, bg=theme.SURFACE_2, highlightthickness=1, highlightbackground=theme.BORDER, padx=14, pady=12)
        self.header = tk.Frame(self, bg=theme.SURFACE_2)
        self.header.pack(fill="x")
        tk.Label(self.header, text=title, bg=theme.SURFACE_2, fg=theme.TEXT, font=(theme.FONT_FAMILY, 10, "bold"), anchor="w").pack(side="left")
        self.chips = tk.Frame(self, bg=theme.SURFACE_2)
        self.chips.pack(fill="x", pady=(6, 10))
        self.body = tk.Frame(self, bg=theme.SURFACE_2)
        self.body.pack(fill="x")


class _MatrixCard(_Card):
    """One {"scaling_coef", "matrix"} entry, edited in place in the loaded
    JSON (node)."""

    def __init__(self, parent: tk.Widget, name: str, node: dict, editor: "GainsEditor") -> None:
        super().__init__(parent, name)
        self._node, self._editor = node, editor
        # The JSON may store a column as a flat list; it's edited as rows and
        # written back in the same form.
        self._flat = not isinstance(node["matrix"][0], list)
        self._rows = [[value] for value in node["matrix"]] if self._flat else node["matrix"]
        self._expanded = False
        self._content: Optional[tk.Widget] = None

        Chip(self.chips, f"{len(self._rows)}×{len(self._rows[0])}").pack(side="left", padx=(0, 6))
        self._structure_chip = Chip(self.chips, "")
        self._structure_chip.pack(side="left", padx=(0, 16))
        tk.Label(self.chips, text="scaling coef", bg=theme.SURFACE_2, fg=theme.MUTED, font=(theme.FONT_FAMILY, 9)).pack(side="left", padx=(0, 6))
        _NumberField(self.chips, editor, node["scaling_coef"], lambda value: node.update(scaling_coef=value),
                     _field_width([node["scaling_coef"]]), theme.SURFACE_2).pack(side="left")

        self._toggle = tk.Label(self.header, text="", bg=theme.SURFACE_2, fg=theme.RED, cursor="hand2", font=(theme.FONT_FAMILY, 9, "bold"))
        self._toggle.bind("<Button-1>", lambda _e: self._set_expanded(not self._expanded))
        self._draw()

    def _set(self, i: int, j: int, value: Any) -> None:
        self._rows[i][j] = value
        if self._flat:
            self._node["matrix"][i] = value

    def _set_expanded(self, expanded: bool) -> None:
        self._expanded = expanded
        self._draw()
        # The card's size changed, so the flow may need to re-span it.
        self.event_generate("<<CardResized>>", when="tail")

    def _draw(self) -> None:
        if self._content is not None:
            self._content.destroy()
        structure = matrix_structure(self._rows)
        self._structure_chip.configure(text={"identity": "scaled identity", "diagonal": "diagonal", "matrix": "full"}[structure])
        compact = structure != "matrix" and not self._expanded
        if structure == "matrix" and not self._expanded:
            self._toggle.pack_forget()
        else:
            self._toggle.configure(text="Show compact" if self._expanded else "Show full matrix")
            self._toggle.pack(side="right")

        if not compact:
            self._content = self._grid()
        elif structure == "identity":
            self._content = self._identity()
        else:
            self._content = self._diagonal()
        self._content.pack(anchor="w")

    def _identity(self) -> tk.Widget:
        frame = tk.Frame(self.body, bg=theme.SURFACE_2)
        size = len(self._rows)

        def set_diagonal(value: Any) -> None:
            for i in range(size):
                self._set(i, i, value)

        _NumberField(frame, self._editor, self._rows[0][0], set_diagonal, _field_width([self._rows[0][0]]), theme.SURFACE_2).pack(side="left")
        tk.Label(frame, text=f"·  I{str(size).translate(_SUBSCRIPTS)}", bg=theme.SURFACE_2, fg=theme.TEXT, font=(MONO[0], 13)).pack(side="left", padx=(8, 0))
        return frame

    def _diagonal(self) -> tk.Widget:
        frame = tk.Frame(self.body, bg=theme.SURFACE_2)
        diagonal = [self._rows[i][i] for i in range(len(self._rows))]
        width = _field_width(diagonal)
        for index, value in enumerate(diagonal):
            line, column = divmod(index, DIAGONAL_PER_LINE)
            tk.Label(frame, text=f"{index + 1},{index + 1}", bg=theme.SURFACE_2, fg=theme.DIM, font=(MONO[0], 8)).grid(row=2 * line, column=column, sticky="e", padx=2)
            _NumberField(frame, self._editor, value, lambda v, i=index: self._set(i, i, v), width, theme.SURFACE_2).grid(row=2 * line + 1, column=column, padx=2, pady=(0, 6))
        return frame

    # The full matrix, with 1-based row/column numbers. Past MAX_VISIBLE
    # rows/columns it sits in a scrolling viewport so the card stays small.
    def _grid(self) -> tk.Widget:
        n_rows, n_cols = len(self._rows), len(self._rows[0])
        outer = tk.Frame(self.body, bg=theme.SURFACE_2)
        canvas = tk.Canvas(outer, bg=theme.SURFACE_2, highlightthickness=0, borderwidth=0)
        grid = tk.Frame(canvas, bg=theme.SURFACE_2)
        canvas.create_window((0, 0), window=grid, anchor="nw")

        width = _field_width([value for row in self._rows for value in row])
        for j in range(n_cols):
            tk.Label(grid, text=str(j + 1), bg=theme.SURFACE_2, fg=theme.DIM, font=(MONO[0], 8)).grid(row=0, column=j + 1, sticky="e", padx=2)
        for i, row in enumerate(self._rows):
            tk.Label(grid, text=str(i + 1), bg=theme.SURFACE_2, fg=theme.DIM, font=(MONO[0], 8)).grid(row=i + 1, column=0, sticky="e", padx=(0, 6))
            for j, value in enumerate(row):
                _NumberField(grid, self._editor, value, lambda v, i=i, j=j: self._set(i, j, v), width, theme.SURFACE_2).grid(row=i + 1, column=j + 1, padx=2, pady=2)

        # Size the viewport to at most MAX_VISIBLE rows/columns of fields.
        grid.update_idletasks()
        cell_width = (grid.winfo_reqwidth() - grid.grid_bbox(0, 0)[2]) / n_cols
        cell_height = (grid.winfo_reqheight() - grid.grid_bbox(0, 0)[3]) / n_rows
        canvas.configure(
            width=min(grid.winfo_reqwidth(), grid.grid_bbox(0, 0)[2] + MAX_VISIBLE * cell_width),
            height=min(grid.winfo_reqheight(), grid.grid_bbox(0, 0)[3] + MAX_VISIBLE * cell_height),
            scrollregion=(0, 0, grid.winfo_reqwidth(), grid.winfo_reqheight()),
        )
        canvas.grid(row=0, column=0, sticky="nw")
        if n_cols > MAX_VISIBLE:
            x_scroll = ttk.Scrollbar(outer, orient="horizontal", command=canvas.xview, style="Horizontal.TScrollbar")
            x_scroll.grid(row=1, column=0, sticky="ew")
            canvas.configure(xscrollcommand=x_scroll.set)
        if n_rows > MAX_VISIBLE:
            y_scroll = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview, style="Vertical.TScrollbar")
            y_scroll.grid(row=0, column=1, sticky="ns")
            canvas.configure(yscrollcommand=y_scroll.set)
        return outer


class _ValuesCard(_Card):
    """The plain numbers and on/off switches of one section."""

    def __init__(self, parent: tk.Widget, title: str, node: dict, keys: list[str], editor: "GainsEditor") -> None:
        super().__init__(parent, title)
        Chip(self.chips, f"{len(keys)} value{'s' if len(keys) != 1 else ''}").pack(side="left")
        width = _field_width([node[key] for key in keys if not isinstance(node[key], bool)] or [0])
        for row, key in enumerate(keys):
            tk.Label(self.body, text=key, bg=theme.SURFACE_2, fg=theme.MUTED, font=(theme.FONT_FAMILY, 9), anchor="w").grid(row=row, column=0, sticky="w", pady=2)
            value = node[key]
            if isinstance(value, bool):
                var = tk.BooleanVar(value=value)
                var.trace_add("write", lambda *_a, k=key, v=var: (node.update({k: v.get()}), editor.mark_dirty()))
                ttk.Checkbutton(self.body, variable=var, style="Choice.TCheckbutton").grid(row=row, column=1, sticky="e")
            elif isinstance(value, (int, float)):
                _NumberField(self.body, editor, value, lambda v, k=key: node.update({k: v}), width, theme.SURFACE_2).grid(row=row, column=1, sticky="e", padx=(18, 0), pady=2)
            else:
                tk.Label(self.body, text=json.dumps(value), bg=theme.SURFACE_2, fg=theme.DIM, font=MONO).grid(row=row, column=1, sticky="e")
        self.body.columnconfigure(0, weight=1)


class _CardFlow(tk.Frame):
    """Lays cards out left to right in as many columns as the width allows;
    a card wider than one column spans as many columns as it needs."""

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, bg=theme.BG)
        self.cards: list[tk.Widget] = []
        self._layout_key: tuple[int, ...] = ()
        self.bind("<Configure>", lambda _e: self._layout())

    def add(self, card: tk.Widget) -> None:
        self.cards.append(card)
        card.bind("<<CardResized>>", lambda _e: self._layout(force=True))

    def _layout(self, force: bool = False) -> None:
        width = max(self.winfo_width(), CARD_WIDTH)
        columns = max(1, width // CARD_WIDTH)
        spans = tuple(min(columns, math.ceil((card.winfo_reqwidth() + 12) / (width / columns))) for card in self.cards)
        # Only re-grid when the arrangement would actually change; gridding
        # resizes the frame, which would otherwise loop back in here.
        if not force and (columns, *spans) == self._layout_key:
            return
        self._layout_key = (columns, *spans)
        for index in range(columns):
            self.columnconfigure(index, weight=1, uniform="cards")
        row = column = 0
        for card, span in zip(self.cards, spans):
            if column + span > columns:
                row, column = row + 1, 0
            card.grid(row=row, column=column, columnspan=span, sticky="nsew", padx=(0, 12), pady=(0, 12))
            column += span


# --- The editor -------------------------------------------------------------------------
class GainsEditor(ttk.Frame):
    """Loads one gains file, edits it in place, saves it. on_dirty(bool) is
    called whenever the unsaved state changes (for the tab's "*")."""

    def __init__(self, parent: tk.Widget, on_dirty: Callable[[bool], None]) -> None:
        super().__init__(parent, style="App.TFrame", padding=(4, 14, 4, 0))
        self.path: Optional[Path] = None
        self.dirty = False
        self._on_dirty = on_dirty
        self._data: Optional[dict] = None
        self._invalid: set[tk.Widget] = set()

        toolbar = ttk.Frame(self, style="App.TFrame")
        toolbar.pack(fill="x")
        self._title = ttk.Label(toolbar, text="", style="MajorHeading.TLabel")
        self._title.pack(side="left")
        self._save_button = ttk.Button(toolbar, text="Save Gains", style="Primary.TButton", command=self.save)
        self._save_button.pack(side="right")
        ttk.Button(toolbar, text="Reload", style="Secondary.TButton", command=self._on_reload).pack(side="right", padx=(0, 8))
        self._path_label = ttk.Label(self, text="", style="OptionHelp.TLabel")
        self._path_label.pack(anchor="w", pady=(4, 0))
        self._note = ttk.Label(self, text="", style="MajorHeadingNote.TLabel", wraplength=1100, justify="left")
        self._note.pack(anchor="w", pady=(2, 0))
        tk.Frame(self, bg=theme.BORDER, height=1).pack(fill="x", pady=(10, 6))
        self._scroll = ScrollableFrame(self)
        self._scroll.pack(fill="both", expand=True)

    # --- State used by the fields ---------------------------------------------------
    def mark_dirty(self) -> None:
        if not self.dirty:
            self.dirty = True
            self._on_dirty(True)
            self._update_note()

    def mark_invalid(self, field: tk.Widget, invalid: bool) -> None:
        (self._invalid.add if invalid else self._invalid.discard)(field)
        self._update_note()

    # Fields a card has since redrawn (e.g. "Show full matrix") are gone,
    # and so is their invalid text; the matrix keeps its last valid value.
    def _prune_invalid(self) -> None:
        self._invalid = {field for field in self._invalid if field.winfo_exists()}

    def _update_note(self) -> None:
        self._prune_invalid()
        if self._invalid:
            self._note.configure(text=f"{len(self._invalid)} field(s) aren't valid numbers (shown in red); fix them to save.")
        elif self.dirty:
            self._note.configure(text="Unsaved changes.")
        else:
            self._note.configure(text="")

    # --- Loading and saving -----------------------------------------------------------
    # Loads `path` (None to show `message` instead, e.g. no controller picked).
    def load(self, path: Optional[Path], title: str, message: str = "") -> None:
        self.path = path
        self._title.configure(text=title)
        self.reload(message)

    def _on_reload(self) -> None:
        if self.dirty and not dialogs.ask_yes_no(self.winfo_toplevel(), "Discard changes?",
                                                 f"Reload {self.path.name} from disk and discard the unsaved changes?",
                                                 yes="Discard", no="Cancel"):
            return
        self.reload()

    def reload(self, message: str = "") -> None:
        self._scroll.clear()
        self._invalid.clear()
        self._data = None
        if self.dirty:
            self.dirty = False
            self._on_dirty(False)
        self._path_label.configure(text=str(self.path) if self.path else "")
        try:
            if self.path is None:
                raise ValueError(message)
            self._data = json.loads(self.path.read_text())
        except (OSError, ValueError) as error:
            self._save_button.state(["disabled"])
            self._note.configure(text=str(error) if self.path else message)
            return
        self._save_button.state(["!disabled"])
        self._update_note()
        self._render_section(self._scroll.body, self._data, depth=0)

    # Writes the edits back. Returns whether the file was saved.
    def save(self) -> bool:
        if self._data is None or self.path is None:
            return False
        self._prune_invalid()
        if self._invalid:
            dialogs.show_error(self.winfo_toplevel(), "Invalid gains", f"{len(self._invalid)} field(s) aren't valid numbers (shown in red). Fix them before saving.")
            return False
        self.path.write_text(to_json(self._data) + "\n")
        self.dirty = False
        self._on_dirty(False)
        self._update_note()
        return True

    # Renders one level of the JSON: sub-sections become headings, matrices
    # and the section's plain values become cards.
    def _render_section(self, parent: tk.Widget, node: dict, depth: int, name: str = "") -> None:
        flow = _CardFlow(parent)
        values, subsections = [], []
        for key, value in node.items():
            if is_matrix_entry(value):
                flow.add(_MatrixCard(flow, key, value, self))
            elif isinstance(value, dict):
                subsections.append(key)
            else:
                values.append(key)
        if values:
            flow.add(_ValuesCard(flow, f"{name or 'Top-level'} values", node, values, self))
        if flow.cards:
            flow.pack(fill="x")
        else:
            flow.destroy()

        for key in subsections:
            title = key.replace("_", " ").title()
            ttk.Label(parent, text=title, style="MajorHeading.TLabel" if depth == 0 else "SubHeading.TLabel").pack(anchor="w", pady=(16 if depth == 0 else 8, 0))
            if depth == 0:
                tk.Frame(parent, bg=theme.BORDER, height=1).pack(fill="x", pady=(8, 12))
            body = ttk.Frame(parent, style="Section.TFrame", padding=(0 if depth == 0 else 12, 0, 0, 0))
            body.pack(fill="x")
            self._render_section(body, node[key], depth + 1, title)
