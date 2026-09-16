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
# File:        dialogs.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Small reusable dialog helpers, themed to match the rest of the ACSL
#     GUI, for cases the standard tkinter.simpledialog/messagebox don't
#     cover: picking one or several items from a list (optionally with a
#     description preview), and entering a multi-line block of text. Shared
#     across pages rather than owned by Settings, since future pages are
#     likely to want the same "pick one/some of these" or "describe this"
#     prompts (e.g. choosing a mission to run against).
###############################################################################

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Optional

from . import theme


def _center_over_parent(dialog: tk.Toplevel, parent: tk.Misc) -> None:
    dialog.update_idletasks()
    root = parent.winfo_toplevel()
    x = root.winfo_rootx() + max(0, (root.winfo_width() - dialog.winfo_width()) // 2)
    y = root.winfo_rooty() + max(0, (root.winfo_height() - dialog.winfo_height()) // 2)
    dialog.geometry(f"+{x}+{y}")


def _make_listbox(parent: tk.Widget, options: list[str], *, selectmode: str) -> tk.Listbox:
    list_frame = tk.Frame(parent, bg=theme.BORDER)
    list_frame.pack(fill="both", expand=True, padx=18)
    listbox = tk.Listbox(
        list_frame,
        bg=theme.FIELD,
        fg=theme.TEXT,
        selectbackground=theme.RED,
        selectforeground="#ffffff",
        highlightthickness=0,
        borderwidth=0,
        activestyle="none",
        font=(theme.FONT_FAMILY, 10),
        height=min(10, max(3, len(options))),
        width=36,
        selectmode=selectmode,
        exportselection=False,
    )
    listbox.pack(fill="both", expand=True, padx=1, pady=1)
    for option in options:
        listbox.insert("end", option)
    return listbox


def _make_description_preview(parent: tk.Widget) -> ttk.Label:
    label = ttk.Label(parent, text="", style="OptionHelp.TLabel", wraplength=320, justify="left")
    label.pack(anchor="w", padx=18, pady=(8, 0), fill="x")
    return label


def choose_from_list(
    parent: tk.Misc,
    title: str,
    prompt: str,
    options: list[str],
    current: Optional[str] = None,
    descriptions: Optional[dict[str, str]] = None,
    confirm_label: str = "Load",
) -> Optional[str]:
    """Show a modal single-selection list picker (with an optional
    description preview under the list) and return the chosen item, or None
    if cancelled."""
    dialog = tk.Toplevel(parent)
    dialog.title(title)
    dialog.configure(bg=theme.BG)
    dialog.transient(parent.winfo_toplevel())
    dialog.resizable(False, False)

    ttk.Label(dialog, text=prompt, style="OptionLabel.TLabel", wraplength=320, justify="left").pack(anchor="w", padx=18, pady=(18, 10))

    listbox = _make_listbox(dialog, options, selectmode="browse")
    preview = _make_description_preview(dialog) if descriptions is not None else None

    def update_preview(_event: Optional[tk.Event] = None) -> None:
        if preview is None:
            return
        selection = listbox.curselection()
        text = descriptions.get(options[selection[0]], "") if selection else ""
        preview.configure(text=text)

    if current in options:
        index = options.index(current)
        listbox.selection_set(index)
        listbox.see(index)
    else:
        listbox.selection_set(0)
    update_preview()
    listbox.bind("<<ListboxSelect>>", update_preview)

    result: dict[str, Optional[str]] = {"value": None}

    def confirm(_event: Optional[tk.Event] = None) -> None:
        selection = listbox.curselection()
        if selection:
            result["value"] = options[selection[0]]
        dialog.destroy()

    def cancel(_event: Optional[tk.Event] = None) -> None:
        dialog.destroy()

    listbox.bind("<Double-Button-1>", confirm)
    listbox.bind("<Return>", confirm)
    dialog.bind("<Escape>", cancel)
    dialog.protocol("WM_DELETE_WINDOW", cancel)

    actions = ttk.Frame(dialog, style="App.TFrame", padding=(18, 14, 18, 18))
    actions.pack(fill="x")
    ttk.Button(actions, text="Cancel", style="Secondary.TButton", command=cancel).pack(side="right")
    ttk.Button(actions, text=confirm_label, style="Primary.TButton", command=confirm).pack(side="right", padx=(0, 8))

    _center_over_parent(dialog, parent)
    dialog.grab_set()
    listbox.focus_set()
    dialog.wait_window()
    return result["value"]


def choose_multiple_from_list(
    parent: tk.Misc,
    title: str,
    prompt: str,
    options: list[str],
    descriptions: Optional[dict[str, str]] = None,
    confirm_label: str = "Delete",
) -> Optional[list[str]]:
    """Show a modal multi-selection list picker and return the chosen items,
    or None if cancelled / nothing was chosen."""
    dialog = tk.Toplevel(parent)
    dialog.title(title)
    dialog.configure(bg=theme.BG)
    dialog.transient(parent.winfo_toplevel())
    dialog.resizable(False, False)

    ttk.Label(dialog, text=prompt, style="OptionLabel.TLabel", wraplength=320, justify="left").pack(anchor="w", padx=18, pady=(18, 10))

    listbox = _make_listbox(dialog, options, selectmode="extended")
    preview = _make_description_preview(dialog) if descriptions is not None else None

    def update_preview(_event: Optional[tk.Event] = None) -> None:
        if preview is None:
            return
        selection = listbox.curselection()
        if len(selection) == 1:
            preview.configure(text=descriptions.get(options[selection[0]], ""))
        elif len(selection) > 1:
            preview.configure(text=f"{len(selection)} missions selected.")
        else:
            preview.configure(text="")

    update_preview()
    listbox.bind("<<ListboxSelect>>", update_preview)

    result: dict[str, Optional[list[str]]] = {"value": None}

    def confirm(_event: Optional[tk.Event] = None) -> None:
        selection = listbox.curselection()
        if selection:
            result["value"] = [options[i] for i in selection]
        dialog.destroy()

    def cancel(_event: Optional[tk.Event] = None) -> None:
        dialog.destroy()

    dialog.bind("<Escape>", cancel)
    dialog.protocol("WM_DELETE_WINDOW", cancel)

    actions = ttk.Frame(dialog, style="App.TFrame", padding=(18, 14, 18, 18))
    actions.pack(fill="x")
    ttk.Button(actions, text="Cancel", style="Secondary.TButton", command=cancel).pack(side="right")
    ttk.Button(actions, text=confirm_label, style="Primary.TButton", command=confirm).pack(side="right", padx=(0, 8))

    _center_over_parent(dialog, parent)
    dialog.grab_set()
    listbox.focus_set()
    dialog.wait_window()
    return result["value"]


def prompt_text(parent: tk.Misc, title: str, prompt: str, initial: str = "") -> Optional[str]:
    """Show a modal multi-line text entry dialog. Returns the entered text
    (which may be empty), or None if cancelled."""
    dialog = tk.Toplevel(parent)
    dialog.title(title)
    dialog.configure(bg=theme.BG)
    dialog.transient(parent.winfo_toplevel())
    dialog.resizable(False, False)

    ttk.Label(dialog, text=prompt, style="OptionLabel.TLabel", wraplength=360, justify="left").pack(anchor="w", padx=18, pady=(18, 10))

    text_border = tk.Frame(dialog, bg=theme.BORDER)
    text_border.pack(fill="both", expand=True, padx=18)
    text_widget = tk.Text(
        text_border,
        bg=theme.FIELD,
        fg=theme.TEXT,
        insertbackground=theme.TEXT,
        borderwidth=0,
        highlightthickness=0,
        font=(theme.FONT_FAMILY, 10),
        wrap="word",
        height=5,
        width=44,
    )
    text_widget.pack(fill="both", expand=True, padx=1, pady=1)
    if initial:
        text_widget.insert("1.0", initial)

    result: dict[str, Optional[str]] = {"value": None}

    def confirm() -> None:
        result["value"] = text_widget.get("1.0", "end-1c").strip()
        dialog.destroy()

    def cancel(_event: Optional[tk.Event] = None) -> None:
        dialog.destroy()

    dialog.bind("<Escape>", cancel)
    dialog.protocol("WM_DELETE_WINDOW", cancel)

    actions = ttk.Frame(dialog, style="App.TFrame", padding=(18, 14, 18, 18))
    actions.pack(fill="x")
    ttk.Button(actions, text="Cancel", style="Secondary.TButton", command=cancel).pack(side="right")
    ttk.Button(actions, text="OK", style="Primary.TButton", command=confirm).pack(side="right", padx=(0, 8))

    _center_over_parent(dialog, parent)
    dialog.grab_set()
    text_widget.focus_set()
    dialog.wait_window()
    return result["value"]
