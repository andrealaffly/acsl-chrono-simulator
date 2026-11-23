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
# File:        simulator.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     The Simulator page: dashboard-style Build / Clean / Run / Run Wrapper
#     controls for the compiled simulator in build/, plus a live console and
#     a Quit Simulation control that's only enabled while build/acsl_sim (Run)
#     or scripts/wrapper.py (Run Wrapper) is actually running.
#
#       Build        - runs `make` in build/.
#       Clean        - deletes every generated file/directory under build/
#                      except .gitignore and .gitkeep (asks for confirmation
#                      first).
#       Run          - launches build/acsl_sim directly, for a single sim.
#       Run Wrapper  - launches scripts/wrapper.py, which itself launches
#                      build/acsl_sim some number of times per
#                      config/wrapper-config.yaml (see that script for the
#                      concurrency/rolling-run details); this page just runs
#                      it as one more subprocess and shows its output.
#       Quit         - sends SIGINT (as if Ctrl+C) to whichever of the above
#                      is currently running; disabled otherwise. Wrapper.py
#                      handles SIGINT itself (stopping its child sims and
#                      closing out the session marker), so this is the same
#                      "ask it to shut down cleanly" signal either way.
#
#     Build/Clean/Run/Run Wrapper all share one "is something running" state
#     so they can't step on each other, and the running subprocess (if any)
#     is stopped when the app window closes (see Page.on_close).
###############################################################################

from __future__ import annotations

import os
import shutil
import signal
import sys
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Optional

from .. import dialogs, theme
from ..console import ProcessConsole
from .base import Page


class _ActionCard(tk.Frame):
    """A clickable dashboard tile: bold title + small description, with a
    hover highlight and a disabled state. Plain tk (not ttk) so its colors
    can be driven directly for the hover/disabled states."""

    def __init__(self, parent: tk.Widget, title: str, description: str, command) -> None:
        super().__init__(
            parent,
            bg=theme.SURFACE_2,
            highlightthickness=1,
            highlightbackground=theme.BORDER,
            highlightcolor=theme.BORDER,
            cursor="hand2",
            padx=18,
            pady=16,
        )
        self._command = command
        self._enabled = True

        self.title_label = tk.Label(self, text=title, bg=theme.SURFACE_2, fg=theme.TEXT, font=(theme.FONT_FAMILY, 13, "bold"), anchor="w")
        self.title_label.pack(anchor="w", fill="x")
        self.desc_label = tk.Label(self, text=description, bg=theme.SURFACE_2, fg=theme.DIM, font=(theme.FONT_FAMILY, 9), anchor="w", justify="left", wraplength=220)
        self.desc_label.pack(anchor="w", fill="x", pady=(5, 0))

        for widget in (self, self.title_label, self.desc_label):
            widget.bind("<Button-1>", self._on_click)
            widget.bind("<Enter>", self._on_enter)
            widget.bind("<Leave>", self._on_leave)

    def _on_click(self, _event: Optional[tk.Event] = None) -> None:
        if self._enabled:
            self._command()

    def _on_enter(self, _event: Optional[tk.Event] = None) -> None:
        if self._enabled:
            self._paint(theme.BORDER)

    def _on_leave(self, _event: Optional[tk.Event] = None) -> None:
        if self._enabled:
            self._paint(theme.SURFACE_2)

    def _paint(self, bg: str) -> None:
        self.configure(bg=bg)
        self.title_label.configure(bg=bg)
        self.desc_label.configure(bg=bg)

    def set_enabled(self, enabled: bool, *, accent_when_enabled: bool = False) -> None:
        self._enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        if not enabled:
            self._paint(theme.SURFACE_2)
            self.title_label.configure(fg=theme.DIM)
        else:
            self.title_label.configure(fg=theme.RED if accent_when_enabled else theme.TEXT)


class SimulatorPage(Page):
    key = "simulator"
    label = "Simulator"

    def __init__(self, shell, repo_dir: Path) -> None:
        super().__init__(shell)
        self.repo_dir = repo_dir
        self.build_dir = repo_dir / "build"
        self.wrapper_script = repo_dir / "scripts" / "wrapper.py"
        self._active_kind: Optional[str] = None  # "build" | "run" | "wrapper" | None
        self._cards: dict[str, _ActionCard] = {}
        self._console: Optional[ProcessConsole] = None

    # --- Page interface ---------------------------------------------------
    def build(self, parent: ttk.Frame) -> tk.Widget:
        root = ttk.Frame(parent, style="App.TFrame")

        ttk.Label(root, text="SIMULATOR", style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(root, text=f"Build, run, and manage {self.build_dir}", style="Body.TLabel").pack(anchor="w", pady=(2, 16))

        actions = ttk.Frame(root, style="App.TFrame")
        actions.pack(fill="x", pady=(0, 16))
        for column in range(5):
            actions.columnconfigure(column, weight=1, uniform="cards")

        self._cards["build"] = _ActionCard(actions, "Build", "Run make in build/.", self._on_build)
        self._cards["clean"] = _ActionCard(actions, "Clean", "Delete generated files in build/ (keeps .gitignore and .gitkeep).", self._on_clean)
        self._cards["run"] = _ActionCard(actions, "Run", "Launch build/acsl_sim.", self._on_run)
        self._cards["wrapper"] = _ActionCard(actions, "Run Wrapper", "Launch scripts/wrapper.py.", self._on_wrapper)
        self._cards["quit"] = _ActionCard(actions, "Quit Simulation", "Stop the running simulator or wrapper (Ctrl+C).", self._on_quit)
        self._cards["quit"].set_enabled(False)

        for index, key in enumerate(["build", "clean", "run", "wrapper", "quit"]):
            self._cards[key].grid(row=0, column=index, sticky="nsew", padx=(0 if index == 0 else 8, 0))

        ttk.Label(root, text="Console", style="SubHeading.TLabel").pack(anchor="w", pady=(4, 6))

        self._console = ProcessConsole(root, self.shell.root)
        self._console.frame.pack(fill="both", expand=True)

        return root

    def has_unsaved_changes(self) -> bool:
        return self._active_kind is not None

    def on_close(self) -> None:
        if self._console is not None:
            self._console.send_signal()

    def _append_console(self, text: str) -> None:
        if self._console is not None:
            self._console.write(text)

    # --- Button state machine ------------------------------------------------
    def _set_busy(self, kind: Optional[str]) -> None:
        self._active_kind = kind
        idle = kind is None
        self._cards["build"].set_enabled(idle)
        self._cards["clean"].set_enabled(idle)
        self._cards["run"].set_enabled(idle)
        self._cards["wrapper"].set_enabled(idle)
        # Quit is only meaningful for Run/Run Wrapper (stopping Build/Clean
        # partway through could leave build/ in a broken state).
        self._cards["quit"].set_enabled(kind in ("run", "wrapper"), accent_when_enabled=True)

    # --- Build ---------------------------------------------------------------
    def _on_build(self) -> None:
        if self._active_kind is not None:
            return
        if not self.build_dir.is_dir():
            dialogs.show_error(self.shell.root, "Build folder missing", f"{self.build_dir} does not exist.")
            return
        self._set_busy("build")
        self.shell.set_status("Building...")

        # Clean removes CMakeCache.txt along with everything else, so the
        # next build needs to re-run cmake before make has anything to build.
        steps = []
        if not (self.build_dir / "CMakeCache.txt").is_file():
            steps.append(["cmake", ".."])
        steps.append(["make", f"-j{os.cpu_count() or 1}"])
        self._console.run_steps(steps, self.build_dir, self._on_build_exit)

    def _on_build_exit(self, returncode: int) -> None:
        self._set_busy(None)
        if returncode == 0:
            self._append_console("\nBuild finished successfully.\n")
            self.shell.set_status("Build finished successfully.")
        else:
            self._append_console(f"\nBuild failed (exit code {returncode}).\n")
            self.shell.set_status(f"Build failed (exit code {returncode}).")

    # --- Clean ---------------------------------------------------------------
    def _on_clean(self) -> None:
        if self._active_kind is not None:
            return
        if not self.build_dir.is_dir():
            dialogs.show_error(self.shell.root, "Build folder missing", f"{self.build_dir} does not exist.")
            return
        if not dialogs.ask_yes_no(
            self.shell.root,
            "Clean build folder",
            f"Delete every generated file in {self.build_dir}, keeping only .gitignore and .gitkeep?\n\nThis cannot be undone.",
            yes="Clean",
            no="Cancel",
        ):
            return

        keep = {".gitignore", ".gitkeep"}
        removed = 0
        try:
            for entry in self.build_dir.iterdir():
                if entry.name in keep:
                    continue
                if entry.is_dir() and not entry.is_symlink():
                    shutil.rmtree(entry)
                else:
                    entry.unlink()
                removed += 1
        except OSError as error:
            self._append_console(f"\nClean failed: {error}\n")
            dialogs.show_error(self.shell.root, "Clean failed", str(error))
            return

        self._append_console(f"\nCleaned {self.build_dir} ({removed} item(s) removed).\n")
        self.shell.set_status("Build folder cleaned.")

    # --- Run / Quit -----------------------------------------------------
    def _on_run(self) -> None:
        if self._active_kind is not None:
            return
        executable = self.build_dir / "acsl_sim"
        if not executable.is_file():
            dialogs.show_error(self.shell.root, "Executable missing", f"{executable} was not found. Build the simulator first.")
            return
        self._set_busy("run")
        self.shell.set_status("Simulator running...")
        self._console.start(["./acsl_sim"], self.build_dir, self._on_run_exit)

    def _on_run_exit(self, returncode: int) -> None:
        self._set_busy(None)
        self._append_console(f"\nSimulator exited (code {returncode}).\n")
        self.shell.set_status("Simulator stopped.")

    # --- Wrapper ---------------------------------------------------------
    def _on_wrapper(self) -> None:
        if self._active_kind is not None:
            return
        if not self.wrapper_script.is_file():
            dialogs.show_error(self.shell.root, "Wrapper script missing", f"{self.wrapper_script} was not found.")
            return
        self._set_busy("wrapper")
        self.shell.set_status("Wrapper running...")
        # Run with the same interpreter this GUI is running under, so it
        # picks up whatever environment/venv already has PyYAML available.
        self._console.start([sys.executable, str(self.wrapper_script)], self.repo_dir, self._on_wrapper_exit)

    def _on_wrapper_exit(self, returncode: int) -> None:
        self._set_busy(None)
        self._append_console(f"\nWrapper exited (code {returncode}).\n")
        self.shell.set_status("Wrapper stopped.")

    # --- Quit (Run or Run Wrapper) -----------------------------------------
    def _on_quit(self) -> None:
        if self._active_kind not in ("run", "wrapper"):
            return
        target = "wrapper" if self._active_kind == "wrapper" else "simulator"
        self._append_console(f"\nSending interrupt to {target} (Ctrl+C)...\n")
        self.shell.set_status(f"Stopping {target}...")
        self._console.send_signal(signal.SIGINT)
