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
#     The Simulator page: dashboard-style Build / Clean / Run controls for
#     the compiled simulator in build/, plus a live console and a Quit
#     Simulation control that's only enabled while build/acsl_sim is
#     actually running.
#
#       Build  - runs `make` in build/.
#       Clean  - deletes every generated file/directory under build/ except
#                .gitignore and .gitkeep (asks for confirmation first).
#       Run    - launches build/acsl_sim.
#       Quit   - sends SIGINT (as if Ctrl+C) to the running acsl_sim process;
#                disabled unless Run is currently active.
#
#     Build/Clean/Run all share one "is something running" state so they
#     can't step on each other, and the running subprocess (if any) is
#     stopped when the app window closes (see Page.on_close).
###############################################################################

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from queue import Empty, Queue
from tkinter import messagebox, ttk
from typing import Optional

from .. import theme
from ..ansi import AnsiConsole
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

    POLL_INTERVAL_MS = 15
    MAX_POLL_WORK_MS = 20

    def __init__(self, shell, repo_dir: Path) -> None:
        super().__init__(shell)
        self.repo_dir = repo_dir
        self.build_dir = repo_dir / "build"
        self._process: Optional[subprocess.Popen] = None
        self._active_kind: Optional[str] = None  # "build" | "run" | None
        self._output_queue: "Queue[object]" = Queue()
        self._cards: dict[str, _ActionCard] = {}
        self._console: Optional[tk.Text] = None
        self._ansi_console: Optional[AnsiConsole] = None

    # --- Page interface ---------------------------------------------------
    def build(self, parent: ttk.Frame) -> tk.Widget:
        root = ttk.Frame(parent, style="App.TFrame")

        ttk.Label(root, text="SIMULATOR", style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(root, text=f"Build, run, and manage {self.build_dir}", style="Body.TLabel").pack(anchor="w", pady=(2, 16))

        actions = ttk.Frame(root, style="App.TFrame")
        actions.pack(fill="x", pady=(0, 16))
        for column in range(4):
            actions.columnconfigure(column, weight=1, uniform="cards")

        self._cards["build"] = _ActionCard(actions, "Build", "Run make in build/.", self._on_build)
        self._cards["clean"] = _ActionCard(actions, "Clean", "Delete generated files in build/ (keeps .gitignore and .gitkeep).", self._on_clean)
        self._cards["run"] = _ActionCard(actions, "Run", "Launch build/acsl_sim.", self._on_run)
        self._cards["quit"] = _ActionCard(actions, "Quit Simulation", "Stop the running simulator (Ctrl+C).", self._on_quit)
        self._cards["quit"].set_enabled(False)

        for index, key in enumerate(["build", "clean", "run", "quit"]):
            self._cards[key].grid(row=0, column=index, sticky="nsew", padx=(0 if index == 0 else 8, 0))

        ttk.Label(root, text="Console", style="SubHeading.TLabel").pack(anchor="w", pady=(4, 6))

        console_border = tk.Frame(root, bg=theme.BORDER)
        console_border.pack(fill="both", expand=True)
        self._console = tk.Text(
            console_border,
            bg=theme.FIELD,
            fg=theme.TEXT,
            insertbackground=theme.TEXT,
            borderwidth=0,
            highlightthickness=0,
            font=("DejaVu Sans Mono", 9),
            wrap="word",
            state="disabled",
        )
        scrollbar = ttk.Scrollbar(console_border, orient="vertical", command=self._console.yview, style="Vertical.TScrollbar")
        self._console.configure(yscrollcommand=scrollbar.set)
        self._console.pack(side="left", fill="both", expand=True, padx=1, pady=1)
        scrollbar.pack(side="right", fill="y")
        self._ansi_console = AnsiConsole(self._console)

        return root

    def has_unsaved_changes(self) -> bool:
        return self._active_kind is not None

    def on_close(self) -> None:
        self._terminate_process()

    # --- Console ------------------------------------------------------------
    def _append_console(self, text: str) -> None:
        if self._console is None or self._ansi_console is None:
            return
        self._console.configure(state="normal")
        self._ansi_console.write(text)
        self._console.see("end")
        self._console.configure(state="disabled")

    # --- Button state machine ------------------------------------------------
    def _set_busy(self, kind: Optional[str]) -> None:
        self._active_kind = kind
        idle = kind is None
        self._cards["build"].set_enabled(idle)
        self._cards["clean"].set_enabled(idle)
        self._cards["run"].set_enabled(idle)
        self._cards["quit"].set_enabled(kind == "run", accent_when_enabled=True)

    # --- Build ---------------------------------------------------------------
    def _on_build(self) -> None:
        if self._active_kind is not None:
            return
        if not self.build_dir.is_dir():
            messagebox.showerror("Build folder missing", f"{self.build_dir} does not exist.", parent=self.shell.root)
            return
        self._set_busy("build")
        self.shell.set_status("Building...")

        # Clean removes CMakeCache.txt along with everything else, so the
        # next build needs to re-run cmake before make has anything to build.
        steps = []
        if not (self.build_dir / "CMakeCache.txt").is_file():
            steps.append(["cmake", ".."])
        steps.append(["make", f"-j{os.cpu_count() or 1}"])
        self._run_steps(steps, self.build_dir, self._on_build_exit)

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
            messagebox.showerror("Build folder missing", f"{self.build_dir} does not exist.", parent=self.shell.root)
            return
        if not messagebox.askyesno(
            "Clean build folder",
            f"Delete every generated file in {self.build_dir}, keeping only .gitignore and .gitkeep?\n\nThis cannot be undone.",
            parent=self.shell.root,
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
            messagebox.showerror("Clean failed", str(error), parent=self.shell.root)
            return

        self._append_console(f"\nCleaned {self.build_dir} ({removed} item(s) removed).\n")
        self.shell.set_status("Build folder cleaned.")

    # --- Run / Quit -----------------------------------------------------
    def _on_run(self) -> None:
        if self._active_kind is not None:
            return
        executable = self.build_dir / "acsl_sim"
        if not executable.is_file():
            messagebox.showerror("Executable missing", f"{executable} was not found. Build the simulator first.", parent=self.shell.root)
            return
        self._set_busy("run")
        self.shell.set_status("Simulator running...")
        self._start_process(["./acsl_sim"], self.build_dir, self._on_run_exit)

    def _on_run_exit(self, returncode: int) -> None:
        self._set_busy(None)
        self._append_console(f"\nSimulator exited (code {returncode}).\n")
        self.shell.set_status("Simulator stopped.")

    def _on_quit(self) -> None:
        if self._active_kind != "run":
            return
        self._append_console("\nSending interrupt to simulator (Ctrl+C)...\n")
        self.shell.set_status("Stopping simulator...")
        self._terminate_process(signal.SIGINT)

    # --- Subprocess plumbing -------------------------------------------------
    def _run_steps(self, commands: list[list[str]], cwd: Path, on_exit) -> None:
        """Run a sequence of commands in cwd, one after another, stopping (and
        reporting that step's exit code) the first time one fails."""

        def run_step(index: int) -> None:
            if index >= len(commands):
                on_exit(0)
                return

            def step_exit(returncode: int) -> None:
                if returncode != 0:
                    on_exit(returncode)
                else:
                    run_step(index + 1)

            self._start_process(commands[index], cwd, step_exit)

        run_step(0)

    def _start_process(self, command: list[str], cwd: Path, on_exit) -> None:
        if self._ansi_console is not None:
            self._ansi_console.reset()
        self._append_console(f"\n$ {' '.join(command)}   (in {cwd})\n")
        try:
            self._process = subprocess.Popen(
                command,
                cwd=str(cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as error:
            self._append_console(f"error: {error}\n")
            self._set_busy(None)
            return

        process = self._process

        def reader() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                self._output_queue.put(line)
            process.wait()  # reap it; avoids a zombie once it has exited

        threading.Thread(target=reader, daemon=True).start()
        self._poll_output(on_exit)

    def _drain_once(self) -> bool:
        # A chatty simulator can enqueue lines far faster than the Text
        # widget can render them. Draining the whole queue in a single call
        # would run inside one Tk event-loop tick and freeze the UI for as
        # long as output kept arriving. A *line-count* cap isn't enough on
        # its own: colored lines cost several Text.insert() calls each (one
        # per color run), so the same cap that's safe for plain text can
        # still block for a while once color is involved. Capping by a
        # wall-clock work budget instead adapts to that automatically.
        any_output = False
        deadline = time.monotonic() + (self.MAX_POLL_WORK_MS / 1000)
        if self._console is not None:
            self._console.configure(state="normal")
        try:
            while True:
                item = self._output_queue.get_nowait()
                if self._ansi_console is not None:
                    self._ansi_console.write(str(item))
                any_output = True
                if time.monotonic() >= deadline:
                    break
        except Empty:
            pass
        if self._console is not None:
            if any_output:
                self._console.see("end")
            self._console.configure(state="disabled")
        return any_output

    def _poll_output(self, on_exit) -> None:
        self._drain_once()

        # Whether the process has exited is checked directly via poll()
        # rather than waiting for a sentinel at the back of the output
        # queue: a burst of output can leave hundreds of thousands of lines
        # queued ahead of that sentinel, which would mean Quit's effect on
        # the UI (re-enabling Run/Build/Clean, disabling Quit) stays stuck
        # behind fully rendering that backlog -- exactly the "takes a bit
        # before it actually stops" symptom. poll() answers immediately;
        # any remaining queued output keeps draining afterward in the
        # background, uncoupled from the busy/idle state.
        process = self._process
        if process is not None and process.poll() is not None:
            returncode = process.returncode
            self._process = None
            on_exit(returncode)
            if not self._output_queue.empty():
                self.shell.root.after(self.POLL_INTERVAL_MS, self._drain_remaining)
            return
        self.shell.root.after(self.POLL_INTERVAL_MS, lambda: self._poll_output(on_exit))

    def _drain_remaining(self) -> None:
        """Keep flushing leftover queued output after the process has
        already exited and the UI has already gone back to idle."""
        if self._drain_once():
            self.shell.root.after(self.POLL_INTERVAL_MS, self._drain_remaining)

    def _terminate_process(self, sig: Optional[signal.Signals] = None) -> None:
        process = self._process
        if process is None or process.poll() is not None:
            return
        try:
            if sig is not None:
                process.send_signal(sig)
            else:
                process.terminate()
        except ProcessLookupError:
            pass
