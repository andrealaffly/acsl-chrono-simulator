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
# File:        genetic_algorithm.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     The "Genetic Algorithm" page: GA tuning of the active controller's
#     gains from the GUI.
#
#     This file is ONLY the user interface. Every piece of GA logic --
#     validating settings, counting genes, finding the session folder,
#     reading progress, choosing what to apply -- lives in the GA package
#     (python-modules/ga), and this page reaches it through a single module,
#     ga.gui_api. See that module's header for how the two talk.
#
#     Tabs:
#       Gains       every tunable entry of the active controller's gains file
#                   (always the one in chrono-assets), each with a tuning
#                   mode and search range
#       Settings    an editor for config/ga-config.yaml, like the Settings
#                   page's tabs for sim-config.yaml and friends: every value
#                   in the file, with the file's comments as help, saved in
#                   place by "Save ga-config.yaml" (comments untouched).
#                   Missions save and load this file with the others.
#       Pre-flight  the checks a session needs to pass (fix failures on the
#                   Settings page -- nothing here edits a config file)
#       Run         progress, results and the GA's console output
#
#     Buttons:
#       Start   saves any Settings edits (after asking), writes the Gains
#               tab's selections to a session file and runs
#               python-modules/ga/run_ga.py with it in the page's console
#               (a separate process, so the window stays responsive), then
#               polls the session's progress.json once a second.
#       Stop    sends that process Ctrl+C; run_ga.py then stops its
#               simulations and restores the original gains.
#       Apply   copies a tuned result over the active gains file (the
#               original is always kept in baseline_tune/).
###############################################################################

from __future__ import annotations

import json
import signal
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Optional, Union

from .. import dialogs, theme
from ..console import ProcessConsole
from ..widgets import ScrollableFrame
from ..yaml_form import Document, Field, Section, ValueType, humanize, is_valid_value
from .base import Page

# The GA package (python-modules/ga; scripts/gui.py puts it on the import path).
from ga import gui_api as ga

class _GainRow:
    """The widgets' state for one row of the Gains tab: an entry, its mode
    and its range factor."""

    def __init__(self, entry: ga.Entry, mode: str, factor: str) -> None:
        self.entry = entry
        self.mode_var = tk.StringVar(value=mode if mode in entry.modes else ga.OFF)
        self.factor_var = tk.StringVar(value=factor)
        self.genes_label: Optional[tk.Label] = None

    def as_tuple(self) -> tuple[str, str, str]:
        """(path, mode, range factor text), the form ga.gui_api takes rows in."""
        return self.entry.path, self.mode_var.get(), self.factor_var.get()


class GeneticAlgorithmPage(Page):
    key = "genetic_algorithm"
    label = "Genetic Algorithm"

    PROGRESS_INTERVAL_MS = 1000   # how often the Run tab re-reads progress.json
    STOP_WAIT_SECONDS = 20.0      # how long closing the GUI waits for the GA to restore the gains

    def __init__(self, shell, repo_dir: Path) -> None:
        super().__init__(shell)
        self.repo_dir = repo_dir
        self.defaults = ga.load_defaults()               # config/ga-config.yaml
        self.ctx: Optional[ga.SimContext] = None         # the active platform/controller
        self.baseline: Optional[dict] = None             # its gains file, as loaded
        self._rows: list[_GainRow] = []
        # Picked modes/ranges, by entry path. They survive a Reload as long
        # as the controller is the same one.
        self._choices: dict[str, tuple[str, str]] = {}
        self._loaded_label = ""
        self._console: Optional[ProcessConsole] = None
        self._started_at: Optional[float] = None         # time.time() of the last Start
        self._session_dir: Optional[Path] = None         # that Start's session folder, once found
        self._results: list[dict] = []                   # pareto.json "solutions", as shown
        self._pareto: dict = {}                          # pareto.json, as last read
        self._wall_per_run: Optional[float] = None       # seconds one run took last time
        # The Settings tab: config/ga-config.yaml being edited.
        self._config_doc: Optional[Document] = None
        self._config_vars: dict[str, tk.Variable] = {}   # field path -> its widget's variable
        self._config_entries: dict[str, ttk.Entry] = {}  # field path -> its text entry (for error styling)
        self._config_dirty = False

    # --- Page interface ---------------------------------------------------------
    def build(self, parent: ttk.Frame) -> tk.Widget:
        root = ttk.Frame(parent, style="App.TFrame")
        ttk.Label(root, text="GENETIC ALGORITHM", style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(root, text="Tune the active controller's gains on position error, velocity error and effort "
                             "(saturated normalized thrust).", style="Body.TLabel").pack(anchor="w", pady=(2, 10))
        self._build_header(root)
        self._build_buttons(root)

        self._notebook = ttk.Notebook(root)
        self._notebook.pack(fill="both", expand=True, pady=(10, 0))
        self._tabs = {}
        for name in ("Gains", "Settings", "Pre-flight", "Run"):
            self._tabs[name] = ttk.Frame(self._notebook, style="App.TFrame", padding=(0, 10, 0, 0))
            self._notebook.add(self._tabs[name], text=name)
        self._build_gains_tab(self._tabs["Gains"])
        self._settings_scroll = ScrollableFrame(self._tabs["Settings"])
        self._settings_scroll.pack(fill="both", expand=True)
        self._preflight_scroll = ScrollableFrame(self._tabs["Pre-flight"])
        self._preflight_scroll.pack(fill="both", expand=True)
        self._build_run_tab(self._tabs["Run"])

        self._refresh(reload_config=True)
        return root

    def on_show(self) -> None:
        # Another page (Settings, a mission load) may have changed the
        # active controller or its gains, so reload whenever shown. Unsaved
        # Settings-tab edits are kept.
        if not self._running:
            self._refresh()

    def has_unsaved_changes(self) -> bool:
        return self._running or self._config_dirty

    # Mission hooks (see Page): Save Mission includes unsaved Settings-tab
    # edits; Load Mission replaces them with the mission's ga-config.yaml.
    def save_config(self) -> bool:
        return self._save_config()

    def reload_config(self) -> None:
        if self._config_doc is not None:   # built yet?
            self._load_config()

    def on_close(self) -> None:
        """The GA must get to restore the original gains before the GUI goes
        away, so this asks it to stop and waits (briefly) for it to wind down."""
        if self._running:
            self._console.send_signal(signal.SIGINT)
            if not self._console.wait(self.STOP_WAIT_SECONDS):
                self._console.send_signal()

    @property
    def _running(self) -> bool:
        return self._console is not None and self._console.running

    # --- Header and buttons -----------------------------------------------------
    def _build_header(self, root: ttk.Frame) -> None:
        box = tk.Frame(root, bg=theme.SURFACE_2, highlightthickness=1, highlightbackground=theme.BORDER, padx=16, pady=12)
        box.pack(fill="x")
        self._header = {}
        for row, (name, title) in enumerate((("target", "Platform / controller"), ("gains", "Gains file"),
                                             ("columns", "Log columns"), ("estimate", "Estimated runtime"))):
            tk.Label(box, text=title, bg=theme.SURFACE_2, fg=theme.DIM, font=(theme.FONT_FAMILY, 9), anchor="w").grid(row=row, column=0, sticky="w", padx=(0, 16))
            self._header[name] = tk.Label(box, text="--", bg=theme.SURFACE_2, fg=theme.TEXT, anchor="w", justify="left",
                                          font=(theme.FONT_FAMILY, 10, "bold" if name == "target" else "normal"))
            self._header[name].grid(row=row, column=1, sticky="w")

    def _build_buttons(self, root: ttk.Frame) -> None:
        bar = ttk.Frame(root, style="App.TFrame")
        bar.pack(fill="x", pady=(10, 0))
        self._buttons = {
            "start": ttk.Button(bar, text="Start", style="Primary.TButton", command=self._on_start),
            "stop": ttk.Button(bar, text="Stop", style="Secondary.TButton", command=self._on_stop),
            "check": ttk.Button(bar, text="Run Pre-flight", style="Secondary.TButton", command=self._on_check),
            "reload": ttk.Button(bar, text="Reload", style="Secondary.TButton", command=self._on_reload),
            "apply": ttk.Button(bar, text="Apply Tuned Gains...", style="Secondary.TButton", command=self._on_apply),
        }
        for name in ("start", "stop", "check", "reload"):
            self._buttons[name].pack(side="left", padx=(0, 8))
        self._buttons["apply"].pack(side="right")
        self._set_running(False)

    def _set_running(self, running: bool) -> None:
        """While a session runs only Stop is enabled; otherwise everything but Stop."""
        for name, button in self._buttons.items():
            button.state(["!disabled"] if running == (name == "stop") else ["disabled"])

    # --- Gains tab --------------------------------------------------------------
    def _build_gains_tab(self, tab: ttk.Frame) -> None:
        top = ttk.Frame(tab, style="App.TFrame")
        top.pack(fill="x", pady=(0, 6))
        self._genes_label = ttk.Label(top, text="", style="SubHeading.TLabel")
        self._genes_label.pack(side="left")
        ttk.Button(top, text="Clear All", style="Secondary.TButton", command=lambda: [r.mode_var.set(ga.OFF) for r in self._rows]).pack(side="right")
        ttk.Label(top, text="Range: each gene may move between value / factor and value x factor (sign kept).   ",
                  style="OptionHelp.TLabel").pack(side="right")
        self._gains_scroll = ScrollableFrame(tab)
        self._gains_scroll.pack(fill="both", expand=True)

    def _populate_gains(self) -> None:
        """One row per entry of the loaded gains file: tunable entries get a
        mode dropdown and a range field; the rest say why they can't be tuned."""
        self._gains_scroll.clear()
        body = self._gains_scroll.body
        self._rows = []
        if self.baseline is None:
            ttk.Label(body, text="No gains file loaded.", style="OptionHelp.TLabel").grid(row=0, column=0, sticky="w")
            self._update_gene_count()
            return

        for column, heading in enumerate(("Entry", "Type", "Mode", "Range factor", "Genes")):
            ttk.Label(body, text=heading, style="SubHeading.TLabel").grid(row=0, column=column, sticky="w", padx=(0, 16), pady=(0, 6))
        body.columnconfigure(0, weight=1)
        default_factor = str(self.defaults.get("default_bound_factor", 10.0))

        for index, entry in enumerate(ga.discover_entries(self.baseline, self.defaults.get("exclude_patterns") or []), start=1):
            tunable = bool(entry.modes) and not entry.excluded
            ttk.Label(body, text=entry.path, style="OptionLabel.TLabel" if tunable else "OptionHelp.TLabel").grid(row=index, column=0, sticky="w", padx=(0, 16), pady=2)
            ttk.Label(body, text=entry.summary, style="OptionHelp.TLabel").grid(row=index, column=1, sticky="w", padx=(0, 16))
            if not tunable:
                ttk.Label(body, text="excluded" if entry.excluded else entry.note, style="OptionHelp.TLabel").grid(row=index, column=2, columnspan=3, sticky="w")
                continue
            row = _GainRow(entry, *self._choices.get(entry.path, (ga.OFF, default_factor)))
            ttk.Combobox(body, textvariable=row.mode_var, values=[ga.OFF] + entry.modes, state="readonly", width=12,
                         style="Field.TCombobox").grid(row=index, column=2, sticky="w", padx=(0, 16))
            ttk.Entry(body, textvariable=row.factor_var, width=8, style="Field.TEntry").grid(row=index, column=3, sticky="w", padx=(0, 16))
            row.genes_label = tk.Label(body, text="", bg=theme.BG, fg=theme.MUTED, font=(theme.FONT_FAMILY, 9), anchor="w")
            row.genes_label.grid(row=index, column=4, sticky="w")
            for var in (row.mode_var, row.factor_var):
                var.trace_add("write", lambda *_a, r=row: self._on_row_changed(r))
            self._rows.append(row)
            self._update_row(row)
        self._update_gene_count()

    def _on_row_changed(self, row: _GainRow) -> None:
        self._choices[row.entry.path] = (row.mode_var.get(), row.factor_var.get())
        self._update_row(row)
        self._update_gene_count()

    def _update_row(self, row: _GainRow) -> None:
        """Show a row's gene count, or what's wrong with it."""
        genes = ga.gene_count(self.baseline, *row.as_tuple())
        if genes is None:
            row.genes_label.configure(text="range factor must be a number > 1", fg=theme.RED)
        else:
            mode = row.mode_var.get()
            row.genes_label.configure(text="" if mode == ga.OFF else f"{genes}  ({ga.MODE_DESCRIPTIONS[mode]})", fg=theme.TEXT)

    def _update_gene_count(self) -> None:
        total = sum(ga.gene_count(self.baseline, *row.as_tuple()) or 0 for row in self._rows)
        entries = sum(row.mode_var.get() != ga.OFF for row in self._rows)
        self._genes_label.configure(text=f"{total} gene(s) selected across {entries} entr{'y' if entries == 1 else 'ies'}")

    # --- Settings tab: editing config/ga-config.yaml ----------------------------
    def _load_config(self) -> None:
        """(Re)read config/ga-config.yaml into the Settings tab, dropping any
        unsaved edits."""
        self._settings_scroll.clear()
        self._config_vars, self._config_entries = {}, {}
        body = self._settings_scroll.body
        try:
            self._config_doc = Document(ga.GA_CONFIG_PATH)
        except OSError as error:
            self._config_doc = None
            ttk.Label(body, text=f"Could not read {ga.GA_CONFIG_PATH}: {error}", style="OptionHelp.TLabel").pack(anchor="w")
            self._set_config_dirty(False)
            return

        ttk.Label(body, text=f"Editing {ga.GA_CONFIG_PATH.relative_to(self.repo_dir)}. Sessions run with the saved file; "
                             "list values (log column names, exclude_patterns) are edited in the file itself.",
                  style="OptionHelp.TLabel", wraplength=820, justify="left").pack(anchor="w", pady=(0, 6))
        for node in self._config_doc.root.children:
            self._render_config_node(body, node, depth=0)
        actions = ttk.Frame(body, style="App.TFrame", padding=(0, 16, 0, 30))
        actions.pack(fill="x")
        ttk.Button(actions, text=f"Save {ga.GA_CONFIG_PATH.name}", style="Secondary.TButton", command=self._save_config).pack(side="left")

        # The runtime estimate follows these three as they're typed.
        for key in ("algorithm.population_size", "algorithm.generations", "runner.concurrent_runs"):
            if key in self._config_vars:
                self._config_vars[key].trace_add("write", lambda *_a: self._update_estimate())
        self._set_config_dirty(False)
        self._update_estimate()

    @staticmethod
    def _is_editable(node: Union[Field, Section]) -> bool:
        """Whether a node is (or contains) a field the tab shows. Sections
        holding only lists (e.g. columns.position_error) have no fields."""
        if isinstance(node, Field):
            return node.path not in ga.NOT_EDITABLE
        return any(GeneticAlgorithmPage._is_editable(child) for child in node.children)

    def _render_config_node(self, parent: tk.Widget, node: Union[Field, Section], depth: int) -> None:
        """A section (heading, help, then its children) or a single field."""
        if not self._is_editable(node):
            return
        if isinstance(node, Field):
            self._render_config_field(parent, node)
            return
        ttk.Label(parent, text=node.title, style="MajorHeading.TLabel" if depth == 0 else "SubHeading.TLabel").pack(anchor="w", pady=(14 if depth == 0 else 8, 2))
        if node.help:
            ttk.Label(parent, text=node.help, style="OptionHelp.TLabel", wraplength=820, justify="left").pack(anchor="w", pady=(0, 4))
        body = ttk.Frame(parent, style="App.TFrame", padding=(18 if depth else 0, 0, 0, 0))
        body.pack(fill="x")
        for child in node.children:
            self._render_config_node(body, child, depth + 1)

    def _render_config_field(self, parent: tk.Widget, field: Field) -> None:
        """One value: a checkbox (booleans), the algorithm dropdown, or a text
        entry, with the comment above it in the file as help."""
        row = ttk.Frame(parent, style="App.TFrame", padding=(0, 4, 0, 4))
        row.pack(fill="x")
        label = humanize(field.key)
        if field.value_type is ValueType.BOOL:
            var: tk.Variable = tk.BooleanVar(value=field.display_value())
            ttk.Checkbutton(row, text=label, variable=var, style="Choice.TCheckbutton").pack(anchor="w")
        else:
            inline = ttk.Frame(row, style="App.TFrame")
            inline.pack(anchor="w", fill="x")
            ttk.Label(inline, text=label, style="OptionLabel.TLabel").pack(side="left")
            shown = field.value_text.strip() if field.value_type in (ValueType.INT, ValueType.FLOAT) else str(field.display_value())
            var = tk.StringVar(value=ga.display_text(field.path, shown))
            if field.path == ga.ALGORITHM_KEY:
                widget = ttk.Combobox(inline, textvariable=var, values=list(ga.ALGORITHMS.values()), state="readonly", width=36, style="Field.TCombobox")
            else:
                width = 12 if field.value_type in (ValueType.INT, ValueType.FLOAT) or field.path == ga.SEED_KEY else 36
                widget = ttk.Entry(inline, textvariable=var, width=width, style="Field.TEntry")
                self._config_entries[field.path] = widget
            widget.pack(side="left", padx=(10, 0))
        if field.comment:
            ttk.Label(row, text=field.comment, style="OptionHelp.TLabel", wraplength=820, justify="left").pack(anchor="w", pady=(2, 0))
        self._config_vars[field.path] = var
        var.trace_add("write", lambda *_a, p=field.path: self._on_config_changed(p))

    def _field_problem(self, path: str) -> Optional[str]:
        """What's wrong with a field's current text, or None."""
        var, field = self._config_vars[path], self._config_doc.fields[path]
        if isinstance(var, tk.BooleanVar):
            return None
        text = var.get()
        if not is_valid_value(text, field.value_type):
            return "must be a whole number" if field.value_type is ValueType.INT else "must be a number"
        return ga.check_setting(path, text)

    def _on_config_changed(self, path: str) -> None:
        entry = self._config_entries.get(path)
        if entry is not None:
            entry.configure(style="FieldInvalid.TEntry" if self._field_problem(path) else "Field.TEntry")
        self._set_config_dirty(True)

    def _set_config_dirty(self, dirty: bool) -> None:
        """Track unsaved edits, shown as "Settings *" on the tab."""
        self._config_dirty = dirty
        self._notebook.tab(self._tabs["Settings"], text="Settings *" if dirty else "Settings")

    def _save_config(self) -> bool:
        """Write the Settings tab to config/ga-config.yaml (only the values
        that changed are rewritten; comments stay as they are). Returns False
        if nothing was saved because a value is invalid."""
        if not self._config_dirty or self._config_doc is None:
            return True
        problems = [f"{humanize(self._config_doc.fields[path].key)} ({path}): {problem}"
                    for path in self._config_vars if (problem := self._field_problem(path))]
        if problems:
            self._notebook.select(self._tabs["Settings"])
            dialogs.show_error(self.shell.root, "Invalid value", f"Fix these in {ga.GA_CONFIG_PATH.name} before saving:\n\n" + "\n".join(problems))
            return False

        for path, var in self._config_vars.items():
            field = self._config_doc.fields[path]
            if isinstance(var, tk.BooleanVar):
                if var.get() != field.display_value():
                    self._config_doc.set_bool(path, var.get())
                continue
            new_text = ga.file_text(path, var.get())
            old_text = field.value_text.strip()
            if field.quoted:
                old_text = old_text[1:-1]
            if new_text != old_text:
                self._config_doc.set_raw(path, new_text)
        self._config_doc.save()
        self.defaults = ga.load_defaults()
        self._set_config_dirty(False)
        self.shell.set_status(f"Saved {ga.GA_CONFIG_PATH.name}.")
        return True

    def _update_estimate(self) -> None:
        label = self._header["estimate"]
        if self._wall_per_run is None:
            label.configure(text="unknown (no previous log of this controller)")
            return
        try:
            runs, concurrent, total = ga.estimate_runtime(self._wall_per_run, self._config_vars["algorithm.population_size"].get(),
                                                          self._config_vars["algorithm.generations"].get(),
                                                          self._config_vars["runner.concurrent_runs"].get())
        except (KeyError, ValueError):
            label.configure(text="--")
            return
        label.configure(text=f"~{ga.format_duration(total)} for {runs} runs "
                             f"({ga.format_duration(self._wall_per_run)} per run alone, {concurrent} at once; parallel runs may each be slower)")

    # --- Pre-flight tab ---------------------------------------------------------
    def _run_preflight(self) -> list[ga.Check]:
        """Run the checks and list them on the Pre-flight tab."""
        _ctx, checks = ga.run_preflight(self.defaults)
        self._preflight_scroll.clear()
        body = self._preflight_scroll.body
        ttk.Label(body, text="Every blocking check must pass before a session can start. Fix failures on the Settings page.",
                  style="OptionHelp.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        for row, check in enumerate(checks, start=1):
            mark, color = ("OK", theme.GREEN) if check.passed else ("FAIL", theme.RED) if check.blocking else ("WARN", theme.AMBER)
            tk.Label(body, text=mark, bg=theme.BG, fg=color, font=(theme.FONT_FAMILY, 9, "bold"), width=5, anchor="w").grid(row=row, column=0, sticky="nw", pady=2)
            tk.Label(body, text=check.label, bg=theme.BG, fg=theme.TEXT, font=(theme.FONT_FAMILY, 10), anchor="w").grid(row=row, column=1, sticky="nw", padx=(0, 16), pady=2)
            tk.Label(body, text=check.detail, bg=theme.BG, fg=theme.DIM, font=(theme.FONT_FAMILY, 9), anchor="w", justify="left",
                     wraplength=700).grid(row=row, column=2, sticky="nw", pady=2)
        body.columnconfigure(2, weight=1)
        return checks

    def _on_check(self) -> None:
        failures = ga.blocking_failures(self._run_preflight())
        self._notebook.select(self._tabs["Pre-flight"])
        self.shell.set_status(f"Pre-flight: {len(failures)} blocking problem(s)." if failures else "Pre-flight passed.")

    # --- Run tab ----------------------------------------------------------------
    def _build_run_tab(self, tab: ttk.Frame) -> None:
        status = ttk.Frame(tab, style="App.TFrame")
        status.pack(fill="x")
        self._run_labels = {}
        for row, (name, title) in enumerate((("state", "State"), ("progress", "Progress"), ("best", "Best so far"),
                                             ("baseline", "Baseline"), ("issues", "Crashed / failed"),
                                             ("time", "Elapsed / remaining"), ("session", "Session folder"))):
            ttk.Label(status, text=title, style="OptionHelp.TLabel").grid(row=row, column=0, sticky="w", padx=(0, 16))
            self._run_labels[name] = ttk.Label(status, text="--", style="OptionLabel.TLabel")
            self._run_labels[name].grid(row=row, column=1, sticky="w")
        self._progressbar = ttk.Progressbar(status, style="Accent.Horizontal.TProgressbar", mode="determinate", maximum=1.0)
        self._progressbar.grid(row=1, column=2, sticky="ew", padx=(16, 0))
        status.columnconfigure(2, weight=1)

        panes = ttk.Frame(tab, style="App.TFrame")
        panes.pack(fill="both", expand=True, pady=(10, 0))
        panes.columnconfigure(0, weight=2)
        panes.columnconfigure(1, weight=3)
        panes.rowconfigure(1, weight=1)
        ttk.Label(panes, text="Results (best first; select one to apply it)", style="SubHeading.TLabel").grid(row=0, column=0, sticky="w", pady=(0, 6))
        ttk.Label(panes, text="Console", style="SubHeading.TLabel").grid(row=0, column=1, sticky="w", padx=(10, 0), pady=(0, 6))

        columns = (("eval", "Candidate", 120), ("score", "Score", 70), ("position", "Position [m]", 90),
                   ("velocity", "Velocity [m/s]", 100), ("effort", "Effort [-]", 80))
        self._results_tree = ttk.Treeview(panes, columns=[c[0] for c in columns], show="headings", style="Results.Treeview", selectmode="browse")
        for column, heading, width in columns:
            self._results_tree.heading(column, text=heading)
            self._results_tree.column(column, width=width, anchor="w")
        self._results_tree.grid(row=1, column=0, sticky="nsew")

        self._console = ProcessConsole(panes, self.shell.root)
        self._console.frame.grid(row=1, column=1, sticky="nsew", padx=(10, 0))

    # --- Loading ----------------------------------------------------------------
    def _on_reload(self) -> None:
        if self._config_dirty and not dialogs.ask_yes_no(
            self.shell.root, "Discard changes?", f"Discard your unsaved changes to {ga.GA_CONFIG_PATH.name}?", yes="Discard", no="Cancel"
        ):
            return
        self._refresh(reload_config=True)

    def _refresh(self, reload_config: bool = False) -> None:
        """Re-read the configs and the active controller's gains file, and
        rebuild the tabs from them. The Settings tab is re-read too, unless
        it holds unsaved edits (reload_config=True re-reads it regardless).
        Also puts back the gains a hard-killed GA session may have left
        behind."""
        if self._running:
            return
        if reload_config or not self._config_dirty:
            self._load_config()
        try:
            message = ga.recover_stale_lock()
            if message:
                self._console.write(message + "\n")
                dialogs.show_warning(self.shell.root, "Gains restored", message)
        except ga.StoreError as error:
            self._console.write(f"{error}\n")

        self.defaults = ga.load_defaults()
        self.baseline = None
        try:
            self.ctx = ga.resolve_context()
        except ga.ConfigError as error:
            self.ctx = None
            for name, text in (("target", f"not available: {error}"), ("gains", "--"), ("columns", "--")):
                self._header[name].configure(text=text)
            self._populate_gains()
            self._run_preflight()
            return

        # A different controller has different entries: forget the old picks.
        if self.ctx.label != self._loaded_label:
            self._choices = {}
        self._loaded_label = self.ctx.label
        self._header["target"].configure(text=self.ctx.label)
        self._header["gains"].configure(text=str(self.ctx.gains_path.relative_to(self.repo_dir)))
        try:
            self.baseline = json.loads(self.ctx.gains_path.read_text())
        except (OSError, ValueError) as error:
            self._header["gains"].configure(text=f"{self.ctx.gains_path}: {error}")

        column_check = ga.column_check(self.ctx, self.defaults)
        self._header["columns"].configure(text=column_check.detail if column_check.passed else f"not GA-ready: {column_check.detail}",
                                          fg=theme.GREEN if column_check.passed else theme.AMBER)
        self._wall_per_run = ga.previous_run_seconds(self.ctx)
        self._update_estimate()
        self._populate_gains()
        self._run_preflight()

    # --- Start / Stop -----------------------------------------------------------
    def _on_start(self) -> None:
        if self._running:
            return
        # Sessions run with the saved ga-config.yaml, so unsaved Settings
        # edits have to be saved (or the Start cancelled) first.
        if self._config_dirty:
            if not dialogs.ask_yes_no(
                self.shell.root, "Save settings?",
                f"Sessions run with the saved {ga.GA_CONFIG_PATH.name}. Save your changes to it and start?",
                yes="Save and Start", no="Cancel",
            ) or not self._save_config():
                return
        try:
            session = ga.build_session([row.as_tuple() for row in self._rows])
        except ValueError as error:
            dialogs.show_error(self.shell.root, "Invalid GA settings", str(error))
            return
        if not session["tune"]:
            self._notebook.select(self._tabs["Gains"])
            dialogs.show_error(self.shell.root, "Nothing to tune", "Pick a mode for at least one entry on the Gains tab.")
            return
        failures = ga.blocking_failures(self._run_preflight())
        if failures or self.ctx is None:
            self._notebook.select(self._tabs["Pre-flight"])
            dialogs.show_error(self.shell.root, "Pre-flight failed",
                               "\n".join(f"•  {c.label}: {c.detail}" for c in failures) or "No active controller.")
            return
        if not dialogs.ask_yes_no(
            self.shell.root,
            "Start GA session",
            f"Tune {self.ctx.label} with {ga.ALGORITHMS.get(str((self.defaults.get('algorithm') or {}).get('name')), '?')}?\n\n"
            f"{self.ctx.gains_path.name} holds candidate gains while the session runs and is restored when it ends "
            f"(a copy is kept in {self.ctx.baseline_dir.name}/).\n\nEstimated runtime: {self._header['estimate'].cget('text')}",
            yes="Start",
            no="Cancel",
        ):
            return

        session_path = ga.write_session_file(self.ctx, session)
        self._results, self._pareto, self._session_dir = [], {}, None
        self._results_tree.delete(*self._results_tree.get_children())
        self._notebook.select(self._tabs["Run"])
        self._started_at = time.time()
        self._set_running(True)
        self.shell.set_status("GA session running...")
        self._console.start(ga.ga_command(session_path), self.repo_dir, self._on_exit)
        self._poll_progress()

    def _on_stop(self) -> None:
        if self._running:
            self._console.write("\nStopping GA (Ctrl+C): finishing up and restoring the original gains...\n")
            self.shell.set_status("Stopping GA session...")
            self._console.send_signal(signal.SIGINT)

    def _on_exit(self, returncode: int) -> None:
        """Called by the console when run_ga.py exits."""
        self._set_running(False)
        self._update_progress()
        message = ga.exit_message(returncode)
        self._console.write(f"\n{message}\n")
        self.shell.set_status(message)

    # --- Progress ---------------------------------------------------------------
    def _poll_progress(self) -> None:
        self._update_progress()
        if self._running:
            self.shell.root.after(self.PROGRESS_INTERVAL_MS, self._poll_progress)

    def _update_progress(self) -> None:
        """Show the session's progress.json and pareto.json on the Run tab."""
        if self._session_dir is None and self.ctx is not None and self._started_at is not None:
            self._session_dir = ga.find_session_dir(self.ctx, self._started_at)
        progress = ga.read_progress(self._session_dir) if self._session_dir else None
        if not progress:
            return
        done, total = progress.get("evaluations_done", 0), progress.get("evaluations_total", 0)
        best, baseline = progress.get("best"), progress.get("baseline")
        texts = {
            "state": {"dry_run": "baseline dry run"}.get(progress.get("state"), progress.get("state", "--")),
            "progress": f"generation {progress.get('generation', 0)}/{progress.get('generations', '?')}, {done}/{total} candidates",
            "issues": f"{progress.get('crashes', 0)} / {progress.get('failures', 0)}",
            "time": f"{ga.format_duration(progress.get('elapsed_seconds'))} / {ga.format_duration(progress.get('eta_seconds'))}",
            "session": str(self._session_dir.relative_to(self.repo_dir)),
        }
        if best and best.get("metrics"):
            m = best["metrics"]
            texts["best"] = f"{best['eval_id']}: score {best['weighted']:.4f} (pos {m['position']:.4f} m, vel {m['velocity']:.4f} m/s, effort {m['effort']:.4f})"
        if baseline:
            texts["baseline"] = (f"pos {baseline['position']:.4f} m, vel {baseline['velocity']:.4f} m/s, "
                                 f"effort {baseline['effort']:.4f}, {baseline['final_time']:.1f} s flown")
        for name, text in texts.items():
            self._run_labels[name].configure(text=text)
        self._progressbar.configure(value=done / total if total else 0.0)
        self._load_results()

    def _load_results(self) -> None:
        """Refill the results table from pareto.json (only if it changed,
        so the user's selection isn't lost on every poll)."""
        pareto = ga.read_pareto(self._session_dir)
        if pareto == self._pareto:
            return
        self._pareto = pareto
        self._results = pareto.get("solutions") or []
        selected = self._results_tree.selection()
        self._results_tree.delete(*self._results_tree.get_children())
        for index, solution in enumerate(self._results):
            m = solution.get("metrics") or {}
            name = solution["eval_id"] + ("  (best)" if solution["eval_id"] == pareto.get("best") else "")
            self._results_tree.insert("", "end", iid=str(index), values=(name, f"{solution['weighted']:.4f}", f"{m.get('position', 0):.4f}",
                                                                         f"{m.get('velocity', 0):.4f}", f"{m.get('effort', 0):.4f}"))
        if selected and self._results_tree.exists(selected[0]):
            self._results_tree.selection_set(selected)

    # --- Apply ------------------------------------------------------------------
    def _apply_source(self) -> Optional[Path]:
        """The file to apply: the selected result (or the session's tuned
        file), else one the user picks from ga_tuned/ and baseline_tune/."""
        selection = self._results_tree.selection()
        solution = self._results[int(selection[0])] if selection else None
        source = ga.resolve_apply_source(self._pareto, solution)
        if source is not None:
            return source

        files = ga.saved_gains_files(self.ctx)
        if not files:
            dialogs.show_info(self.shell.root, "Nothing to apply", f"There are no saved gains in {self.ctx.tuned_dir.name}/ or {self.ctx.baseline_dir.name}/ yet.")
            return None
        descriptions = {name: f"Saved {time.strftime('%Y-%m-%d %H:%M', time.localtime(path.stat().st_mtime))}  ·  {path}" for name, path in files.items()}
        choice = dialogs.choose_from_list(self.shell.root, "Apply gains", "Choose the gains file to copy over the active one.",
                                          list(files), descriptions=descriptions, confirm_label="Choose")
        return files[choice] if choice else None

    def _on_apply(self) -> None:
        if self.ctx is None:
            dialogs.show_error(self.shell.root, "No controller", "No active platform/controller in sim-config.yaml.")
            return
        source = self._apply_source()
        if source is None or not dialogs.ask_yes_no(
            self.shell.root,
            "Apply gains",
            f"Copy\n{source}\n\nover\n{self.ctx.gains_path}?\n\nEvery GA session's starting gains are kept in {self.ctx.baseline_dir}.",
            yes="Apply",
            no="Cancel",
        ):
            return
        try:
            ga.apply_gains_file(self.ctx, source)
        except (OSError, ValueError, ga.StoreError) as error:
            dialogs.show_error(self.shell.root, "Apply failed", str(error))
            return
        self._console.write(f"\nApplied {source} -> {self.ctx.gains_path}\n")
        self.shell.set_status(f"Applied {source.name}.")
        self._refresh()
