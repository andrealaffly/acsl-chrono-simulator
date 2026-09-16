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
# File:        base.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     The interface every ACSL GUI page implements. AppShell (shell.py) only
#     depends on this interface, so adding a new page later (Run, Build,
#     Clean, a flightstack wrapper, ...) means writing one module and adding
#     one line to the PAGES list in gui.py -- no shell changes required.
###############################################################################

from __future__ import annotations

import tkinter as tk
from tkinter import ttk


class Page:
    """Base class for a top-level GUI section, shown as one sidebar entry."""

    # Unique, stable identifier (used internally; not shown to the user).
    key: str = ""
    # Sidebar button text.
    label: str = ""

    def __init__(self, shell) -> None:
        self.shell = shell

    def build(self, parent: ttk.Frame) -> tk.Widget:
        """Build and return this page's content widget, parented under `parent`.

        Called once, the first time the page is shown. The returned widget is
        cached and re-shown (via tkraise) on subsequent visits.
        """
        raise NotImplementedError

    def on_show(self) -> None:
        """Called every time the page becomes visible (after the first build)."""

    def has_unsaved_changes(self) -> bool:
        """Whether the page holds edits/in-progress work that would be lost or
        orphaned by closing the app (unsaved edits, a running subprocess, ...)."""
        return False

    def on_close(self) -> None:
        """Called once, on every page, right before the app window closes.
        Use it to clean up anything that shouldn't outlive the GUI (e.g. a
        subprocess a page started)."""
