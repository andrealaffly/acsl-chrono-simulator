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
# File:        console.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     ProcessConsole: a dark console (tk.Text + scrollbar) that runs one
#     subprocess at a time and streams its output into itself, with ANSI
#     colors rendered (see ansi.py). Shared by the Simulator page (Build /
#     Run / Run Wrapper) and the Genetic Algorithm page (run_ga.py).
#
#     Output is read on a background thread into a queue and drained into the
#     Text widget from the Tk event loop on a time budget, so a chatty process
#     can never freeze the window.
###############################################################################

from __future__ import annotations

import signal
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from queue import Empty, Queue
from tkinter import ttk
from typing import Callable, Optional

from . import theme
from .ansi import AnsiConsole


class ProcessConsole:
    POLL_INTERVAL_MS = 15
    MAX_POLL_WORK_MS = 20

    def __init__(self, parent: tk.Widget, root: tk.Misc) -> None:
        self._root = root
        self._process: Optional[subprocess.Popen] = None
        self._output_queue: "Queue[str]" = Queue()

        # Pack/grid .frame wherever the console should go.
        self.frame = tk.Frame(parent, bg=theme.BORDER)
        self._text = tk.Text(
            self.frame,
            bg=theme.FIELD,
            fg=theme.TEXT,
            insertbackground=theme.TEXT,
            borderwidth=0,
            highlightthickness=0,
            font=("DejaVu Sans Mono", 9),
            wrap="word",
            state="disabled",
        )
        scrollbar = ttk.Scrollbar(self.frame, orient="vertical", command=self._text.yview, style="Vertical.TScrollbar")
        self._text.configure(yscrollcommand=scrollbar.set)
        self._text.pack(side="left", fill="both", expand=True, padx=1, pady=1)
        scrollbar.pack(side="right", fill="y")
        self._ansi = AnsiConsole(self._text)

    @property
    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def write(self, text: str) -> None:
        self._text.configure(state="normal")
        self._ansi.write(text)
        self._text.see("end")
        self._text.configure(state="disabled")

    # --- Running processes ----------------------------------------------------
    # Starts `command` in `cwd`; on_exit(returncode) is called from the Tk
    # event loop once it exits (-1 if it couldn't be started at all).
    def start(self, command: list[str], cwd: Path, on_exit: Callable[[int], None]) -> None:
        # A prior run's unterminated color can't bleed into this one.
        self._ansi.reset()
        self.write(f"\n$ {' '.join(command)}   (in {cwd})\n")
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
            self.write(f"error: {error}\n")
            on_exit(-1)
            return

        process = self._process

        def reader() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                self._output_queue.put(line)
            process.wait()  # reap it; avoids a zombie once it has exited

        threading.Thread(target=reader, daemon=True).start()
        self._poll_output(on_exit)

    # Runs a sequence of commands in cwd, one after another, stopping (and
    # reporting that step's exit code) the first time one fails.
    def run_steps(self, commands: list[list[str]], cwd: Path, on_exit: Callable[[int], None]) -> None:
        def run_step(index: int) -> None:
            if index >= len(commands):
                on_exit(0)
                return

            def step_exit(returncode: int) -> None:
                if returncode != 0:
                    on_exit(returncode)
                else:
                    run_step(index + 1)

            self.start(commands[index], cwd, step_exit)

        run_step(0)

    # SIGINT (Ctrl+C) asks a process to shut down cleanly; no signal means
    # terminate().
    def send_signal(self, sig: Optional[signal.Signals] = None) -> None:
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

    # Blocks until the running process exits, for at most `timeout` seconds.
    # Returns whether it exited.
    def wait(self, timeout: float) -> bool:
        process = self._process
        if process is None:
            return True
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    # --- Output draining -------------------------------------------------------
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
        self._text.configure(state="normal")
        try:
            while True:
                self._ansi.write(self._output_queue.get_nowait())
                any_output = True
                if time.monotonic() >= deadline:
                    break
        except Empty:
            pass
        if any_output:
            self._text.see("end")
        self._text.configure(state="disabled")
        return any_output

    def _poll_output(self, on_exit: Callable[[int], None]) -> None:
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
            self._process = None
            on_exit(process.returncode)
            if not self._output_queue.empty():
                self._root.after(self.POLL_INTERVAL_MS, self._drain_remaining)
            return
        self._root.after(self.POLL_INTERVAL_MS, lambda: self._poll_output(on_exit))

    # Keeps flushing leftover queued output after the process has already
    # exited and the page has already gone back to idle.
    def _drain_remaining(self) -> None:
        if self._drain_once():
            self._root.after(self.POLL_INTERVAL_MS, self._drain_remaining)
