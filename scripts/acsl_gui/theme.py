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
# File:        theme.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Shared color palette and ttk style definitions for the ACSL GUI. Mirrors
#     the dark/red visual identity used by scripts/installer.py so that every
#     page added to the GUI (Settings today; Run/Build/Clean/etc. later) looks
#     like one application.
###############################################################################

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

# --- Palette (matches InstallerUI in installer.py) --------------------------
BG = "#0E0E0E"
SURFACE = "#0E0E0E"
SURFACE_2 = "#181818"
FIELD = "#080808"
TEXT = "#f3f1f1"
MUTED = "#b7b2b2"
DIM = "#787272"
BORDER = "#343030"
RED = "#ff3d35"
RED_ACTIVE = "#e9342e"
RED_DARK = "#7c2324"
GREEN = "#5eea8b"
AMBER = "#fbbf24"

FONT_FAMILY = "DejaVu Sans"


def setup_style(root: tk.Tk) -> ttk.Style:
    """Configure every ttk style shared across the GUI's pages and shell."""
    style = ttk.Style(root)
    if "clam" in style.theme_names():
        style.theme_use("clam")

    # --- Frames / surfaces ---------------------------------------------
    style.configure("App.TFrame", background=BG)
    style.configure("Surface.TFrame", background=SURFACE)
    style.configure("Sidebar.TFrame", background=SURFACE_2)
    # Plain (un-boxed) body frame used for settings form content.
    style.configure("Section.TFrame", background=BG)

    # --- Text -------------------------------------------------------------
    style.configure("Header.TLabel", background=BG, foreground=TEXT, font=(FONT_FAMILY, 21, "bold"))
    style.configure("Tagline.TLabel", background=BG, foreground=RED, font=(FONT_FAMILY, 10, "bold"))
    style.configure("Eyebrow.TLabel", background=SURFACE, foreground=RED, font=(FONT_FAMILY, 9, "bold"))
    style.configure("Title.TLabel", background=SURFACE, foreground=TEXT, font=(FONT_FAMILY, 18, "bold"))
    style.configure("Body.TLabel", background=SURFACE, foreground=MUTED, font=(FONT_FAMILY, 10))
    style.configure("Status.TLabel", background=BG, foreground=MUTED, font=(FONT_FAMILY, 9))

    # --- Settings form hierarchy: major heading > sub heading > option -----
    style.configure("MajorHeading.TLabel", background=BG, foreground=TEXT, font=(FONT_FAMILY, 14, "bold"))
    style.configure("MajorHeadingNote.TLabel", background=BG, foreground=AMBER, font=(FONT_FAMILY, 9))
    style.configure("SubHeading.TLabel", background=BG, foreground=RED, font=(FONT_FAMILY, 11, "bold"))
    style.configure("SubHeadingNote.TLabel", background=BG, foreground=AMBER, font=(FONT_FAMILY, 9))
    style.configure("OptionLabel.TLabel", background=BG, foreground=TEXT, font=(FONT_FAMILY, 10))
    style.configure("OptionHelp.TLabel", background=BG, foreground=DIM, font=(FONT_FAMILY, 9))

    # --- Buttons ------------------------------------------------------------
    style.configure("Primary.TButton", background=RED, foreground="#ffffff", borderwidth=0, font=(FONT_FAMILY, 10, "bold"), padding=(18, 10))
    style.map("Primary.TButton", background=[("active", RED_ACTIVE), ("pressed", RED_DARK)], foreground=[("disabled", "#969090")])
    style.configure("Secondary.TButton", background=SURFACE_2, foreground=TEXT, borderwidth=0, font=(FONT_FAMILY, 10), padding=(18, 10))
    style.map("Secondary.TButton", background=[("active", BORDER), ("pressed", BORDER)])

    # --- Sidebar navigation buttons -----------------------------------------
    style.configure("Nav.TButton", background=SURFACE_2, foreground=MUTED, borderwidth=0, anchor="w", font=(FONT_FAMILY, 10, "bold"), padding=(16, 12))
    style.map("Nav.TButton", background=[("active", BORDER)], foreground=[("active", TEXT)])
    style.configure("NavActive.TButton", background=RED, foreground="#ffffff", borderwidth=0, anchor="w", font=(FONT_FAMILY, 10, "bold"), padding=(16, 12))
    style.map("NavActive.TButton", background=[("active", RED_ACTIVE)])

    # --- Inputs -------------------------------------------------------------
    # clam's checkbutton indicator element doesn't reliably pick up
    # indicatorcolor on every platform, so "checked" is also signalled by the
    # label itself turning the accent color -- a guaranteed-to-render cue.
    style.configure("Choice.TCheckbutton", background=BG, foreground=TEXT, font=(FONT_FAMILY, 10), padding=4, borderwidth=0, relief="flat")
    style.map(
        "Choice.TCheckbutton",
        background=[("active", BG), ("disabled", BG)],
        foreground=[("selected", RED), ("active", TEXT), ("disabled", DIM)],
        indicatorcolor=[("selected", RED), ("!selected", FIELD)],
    )
    style.configure("Field.TEntry", fieldbackground=FIELD, foreground=TEXT, insertcolor=TEXT, borderwidth=1, relief="flat", padding=6)
    style.map("Field.TEntry", fieldbackground=[("disabled", SURFACE_2)])
    style.configure("FieldInvalid.TEntry", fieldbackground=FIELD, foreground=RED, insertcolor=TEXT, borderwidth=1, relief="flat", padding=6)

    style.configure(
        "Field.TCombobox",
        fieldbackground=FIELD,
        background=FIELD,
        foreground=TEXT,
        arrowcolor=MUTED,
        borderwidth=1,
        relief="flat",
        padding=6,
    )
    style.map(
        "Field.TCombobox",
        fieldbackground=[("readonly", FIELD), ("disabled", SURFACE_2)],
        foreground=[("readonly", TEXT)],
        background=[("readonly", FIELD)],
        arrowcolor=[("active", RED)],
    )
    root.option_add("*TCombobox*Listbox.background", FIELD)
    root.option_add("*TCombobox*Listbox.foreground", TEXT)
    root.option_add("*TCombobox*Listbox.selectBackground", RED)
    root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

    # --- Notebook (tabs) ------------------------------------------------
    # Only background/foreground differ on the selected tab -- padding and
    # expand are pinned to the same value in every state so the tab's size
    # never changes, only its highlight color.
    style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 6, 0, 0))
    style.configure("TNotebook.Tab", background=SURFACE_2, foreground=MUTED, font=(FONT_FAMILY, 10, "bold"), padding=(16, 9), borderwidth=0, focuscolor=BG)
    style.map(
        "TNotebook.Tab",
        background=[("selected", RED), ("!selected", SURFACE_2)],
        foreground=[("selected", "#ffffff"), ("!selected", MUTED)],
        padding=[("selected", (16, 9)), ("!selected", (16, 9))],
        expand=[("selected", (0, 0, 0, 0)), ("!selected", (0, 0, 0, 0))],
    )

    # --- Scrollbar --------------------------------------------------------
    style.configure("Vertical.TScrollbar", background=SURFACE_2, troughcolor=BG, bordercolor=BG, arrowcolor=MUTED, relief="flat")

    return style
