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
# File:        shell.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     The ACSL GUI's application shell: a fixed header (logo/title), a left
#     sidebar listing every registered page, a content area that swaps
#     between pages, and a status bar. This is the one place that knows about
#     window chrome; individual pages (scripts/acsl_gui/pages/*.py) only
#     implement the Page interface and are unaware of the shell's layout.
#
#     Adding a future page (Run, Build, Clean, a flightstack wrapper, ...) is
#     done by passing an additional Page instance into AppShell's `pages`
#     list in gui.py -- nothing in this file needs to change.
###############################################################################

from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Optional

from . import theme
from .pages.base import Page

APP_TITLE = "ACSL UAV Simulator"
APP_TAGLINE = "A Project-Chrono Based High Fidelity Simulator for UAVs"
LOGO_MAX_WIDTH = 620


class AppShell:
    WINDOW_WIDTH = 1180
    WINDOW_HEIGHT = 860
    SIDEBAR_WIDTH = 220

    def __init__(self, repo_dir: Path) -> None:
        self.repo_dir = repo_dir
        self.pages: list[Page] = []
        self.root = tk.Tk()
        self.root.withdraw()
        self.root.title(f"{APP_TITLE} — Control Panel")
        self.root.geometry(f"{self.WINDOW_WIDTH}x{self.WINDOW_HEIGHT}")
        self.root.minsize(960, 640)
        self.root.configure(bg=theme.BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._style = theme.setup_style(self.root)
        self._nav_buttons: dict[str, ttk.Button] = {}
        self._page_frames: dict[str, tk.Widget] = {}
        self._built_pages: set[str] = set()
        self._active_key: Optional[str] = None
        self._status_var = tk.StringVar(value="Ready.")
        self._logo_image: Optional[tk.PhotoImage] = None

        self._build_window()
        self._center_window()

    # --- Assets -------------------------------------------------------------
    def _load_logo(self) -> Optional[tk.PhotoImage]:
        logo_path = self.repo_dir / "chrono-assets" / "sim-assets" / "acsl_sim_logo.jpeg"
        if not logo_path.is_file():
            return None
        try:
            image = tk.PhotoImage(file=str(logo_path))
        except tk.TclError:
            return None
        if image.width() > LOGO_MAX_WIDTH:
            factor = max(1, (image.width() + LOGO_MAX_WIDTH - 1) // LOGO_MAX_WIDTH)
            image = image.subsample(factor, factor)
        self._logo_image = image
        return image

    def register_pages(self, pages: list[Page]) -> None:
        """Attach the shell's pages (each already constructed with `shell=self`),
        populate the sidebar, and show the first one. Call once, after
        constructing every Page instance."""
        self.pages = pages
        for page in pages:
            button = ttk.Button(self._nav_container, text=page.label, style="Nav.TButton", command=lambda k=page.key: self.show_page(k))
            button.pack(fill="x", pady=(0, 6))
            self._nav_buttons[page.key] = button
        if pages:
            self.show_page(pages[0].key)
        self.root.deiconify()

    # --- Layout ---------------------------------------------------------
    def _center_window(self) -> None:
        self.root.update_idletasks()
        x = max(0, (self.root.winfo_screenwidth() - self.WINDOW_WIDTH) // 2)
        y = max(0, (self.root.winfo_screenheight() - self.WINDOW_HEIGHT) // 2)
        self.root.geometry(f"{self.WINDOW_WIDTH}x{self.WINDOW_HEIGHT}+{x}+{y}")

    def _build_window(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame")
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer, style="App.TFrame", padding=(26, 16, 26, 10))
        header.pack(fill="x")
        logo = self._load_logo()
        if logo is not None:
            tk.Label(header, image=logo, bg=theme.BG, borderwidth=0).pack(anchor="w")
        else:
            ttk.Label(header, text=APP_TITLE, style="Header.TLabel").pack(anchor="w")
            ttk.Label(header, text=APP_TAGLINE.upper(), style="Tagline.TLabel").pack(anchor="w", pady=(3, 0))

        body = ttk.Frame(outer, style="App.TFrame")
        body.pack(fill="both", expand=True)

        sidebar = ttk.Frame(body, style="Sidebar.TFrame", width=self.SIDEBAR_WIDTH)
        sidebar.pack(side="left", fill="y")
        sidebar.pack_propagate(False)
        self._nav_container = ttk.Frame(sidebar, style="Sidebar.TFrame", padding=(10, 16, 10, 10))
        self._nav_container.pack(fill="both", expand=True)

        content_outer = ttk.Frame(body, style="App.TFrame", padding=(20, 16, 20, 0))
        content_outer.pack(side="left", fill="both", expand=True)
        self.content = ttk.Frame(content_outer, style="App.TFrame")
        self.content.pack(fill="both", expand=True)

        footer = ttk.Frame(outer, style="App.TFrame", padding=(26, 6, 26, 12))
        footer.pack(fill="x")
        ttk.Label(footer, textvariable=self._status_var, style="Status.TLabel").pack(anchor="w")

    # --- Page management ------------------------------------------------
    def show_page(self, key: str) -> None:
        page = next((p for p in self.pages if p.key == key), None)
        if page is None:
            return

        if key not in self._built_pages:
            frame = page.build(self.content)
            frame.grid(row=0, column=0, sticky="nsew")
            self.content.rowconfigure(0, weight=1)
            self.content.columnconfigure(0, weight=1)
            self._page_frames[key] = frame
            self._built_pages.add(key)

        self._page_frames[key].tkraise()
        page.on_show()

        for page_key, button in self._nav_buttons.items():
            button.configure(style="NavActive.TButton" if page_key == key else "Nav.TButton")
        self._active_key = key

    # --- Shared helpers for pages -----------------------------------------
    def set_status(self, message: str) -> None:
        self._status_var.set(message)

    # --- Lifecycle --------------------------------------------------------
    def _on_close(self) -> None:
        unsaved = [p.label for p in self.pages if p.has_unsaved_changes()]
        if unsaved:
            proceed = messagebox.askyesno(
                "Unsaved or in-progress work",
                "This will lose unsaved edits or stop in-progress work on: " + ", ".join(unsaved) + ".\n\nQuit anyway?",
                parent=self.root,
            )
            if not proceed:
                return
        for page in self.pages:
            page.on_close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()
