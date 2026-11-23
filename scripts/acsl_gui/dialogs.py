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
#     Every popup the ACSL GUI shows, themed to match the rest of the app.
#     Tk's own messagebox/simpledialog are deliberately not used: on Linux
#     they come up tiny, in a mismatched system font, and ignore the dark
#     theme. Instead every dialog here is built on one skeleton (_Dialog): a
#     colored eyebrow + title, a body, and a right-aligned button row, at a
#     comfortable reading size.
#
#       show_info / show_warning / show_error   - a message and an OK button
#       ask_yes_no / ask_yes_no_cancel          - confirmations
#       ask_string                              - one line of text
#       prompt_text                             - a multi-line block of text
#       choose_from_list                        - pick one (with an optional
#                                                 description preview)
#       choose_multiple_from_list               - pick several
#
#     Shared across pages rather than owned by one, since every page needs
#     the same "are you sure" / "pick one of these" prompts.
###############################################################################

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Any, Optional

from . import theme

# Dialog layout: wide enough that paths and pre-flight messages wrap into a
# few readable lines instead of a narrow column.
DIALOG_WIDTH = 600
TEXT_WRAP = DIALOG_WIDTH - 2 * 28
PAD = 28

# Eyebrow text and accent color per dialog kind.
_KINDS = {
    "info": ("INFO", theme.GREEN),
    "warning": ("WARNING", theme.AMBER),
    "error": ("ERROR", theme.RED),
    "question": ("CONFIRM", theme.RED),
    "input": ("INPUT", theme.RED),
}


class _Dialog:
    """The shared skeleton: a modal, themed Toplevel with an accent bar,
    eyebrow, title and a button row. Subclass-free on purpose -- callers add
    their body widgets to .body and buttons with add_button(), then run()."""

    def __init__(self, parent: tk.Misc, title: str, kind: str) -> None:
        self.parent = parent
        self.result: Any = None
        eyebrow, color = _KINDS.get(kind, _KINDS["info"])

        self.window = tk.Toplevel(parent)
        self.window.title(title)
        self.window.configure(bg=theme.BG)
        self.window.transient(parent.winfo_toplevel())
        self.window.resizable(False, False)
        self.window.minsize(DIALOG_WIDTH, 0)

        tk.Frame(self.window, bg=color, height=4).pack(fill="x")
        header = tk.Frame(self.window, bg=theme.BG, padx=PAD, pady=0)
        header.pack(fill="x", pady=(20, 0))
        tk.Label(header, text=eyebrow, bg=theme.BG, fg=color, font=(theme.FONT_FAMILY, 9, "bold"), anchor="w").pack(anchor="w")
        tk.Label(header, text=title, bg=theme.BG, fg=theme.TEXT, font=(theme.FONT_FAMILY, 15, "bold"), anchor="w",
                 justify="left", wraplength=TEXT_WRAP).pack(anchor="w", pady=(2, 0))

        self.body = tk.Frame(self.window, bg=theme.BG, padx=PAD)
        self.body.pack(fill="both", expand=True, pady=(14, 0))
        self._buttons = tk.Frame(self.window, bg=theme.BG, padx=PAD)
        self._buttons.pack(fill="x", pady=(22, 24))

        self.window.protocol("WM_DELETE_WINDOW", self.close)

    # A wrapped paragraph of body text.
    def add_message(self, text: str, *, muted: bool = False) -> None:
        tk.Label(self.body, text=text, bg=theme.BG, fg=theme.MUTED if muted else theme.TEXT, font=(theme.FONT_FAMILY, 11),
                 justify="left", anchor="w", wraplength=TEXT_WRAP).pack(anchor="w", fill="x", pady=(0, 6))

    # Buttons are added right to left (the first one added ends up
    # rightmost), so pass the primary action first. `value` is what run()
    # returns when it's clicked; a callable is evaluated at click time (e.g.
    # to read an entry or a list selection).
    def add_button(self, label: str, value: Any, *, primary: bool = False, default: bool = False, cancel: bool = False) -> None:
        def choose(_event: Optional[tk.Event] = None) -> None:
            self.choose(value)

        style = "Primary.TButton" if primary else "Secondary.TButton"
        ttk.Button(self._buttons, text=label, style=style, command=choose).pack(side="right", padx=(8, 0))
        if default:
            self.window.bind("<Return>", choose)
        if cancel:
            self.window.bind("<Escape>", choose)
            self.window.protocol("WM_DELETE_WINDOW", choose)

    def choose(self, value: Any) -> None:
        self.result = value() if callable(value) else value
        self.window.destroy()

    def close(self, _event: Optional[tk.Event] = None) -> None:
        self.window.destroy()

    # Centers over the app window (not the screen, so it stays anchored to
    # the app on a multi-monitor setup), makes it modal and waits for it.
    def run(self, focus: Optional[tk.Widget] = None) -> Any:
        self.window.update_idletasks()
        root = self.parent.winfo_toplevel()
        x = root.winfo_rootx() + max(0, (root.winfo_width() - self.window.winfo_width()) // 2)
        y = root.winfo_rooty() + max(0, (root.winfo_height() - self.window.winfo_height()) // 3)
        self.window.geometry(f"+{x}+{y}")
        self.window.grab_set()
        (focus or self.window).focus_set()
        self.window.wait_window()
        return self.result


# --- Messages and confirmations -------------------------------------------
def _message(parent: tk.Misc, title: str, message: str, kind: str) -> None:
    dialog = _Dialog(parent, title, kind)
    dialog.add_message(message)
    dialog.add_button("OK", None, primary=True, default=True, cancel=True)
    dialog.run()


def show_info(parent: tk.Misc, title: str, message: str) -> None:
    _message(parent, title, message, "info")


def show_warning(parent: tk.Misc, title: str, message: str) -> None:
    _message(parent, title, message, "warning")


def show_error(parent: tk.Misc, title: str, message: str) -> None:
    _message(parent, title, message, "error")


def ask_yes_no(parent: tk.Misc, title: str, message: str, yes: str = "Yes", no: str = "No") -> bool:
    dialog = _Dialog(parent, title, "question")
    dialog.add_message(message)
    dialog.add_button(yes, True, primary=True, default=True)
    dialog.add_button(no, False, cancel=True)
    return bool(dialog.run())


# Returns True/False for yes/no, or None if cancelled.
def ask_yes_no_cancel(parent: tk.Misc, title: str, message: str) -> Optional[bool]:
    dialog = _Dialog(parent, title, "question")
    dialog.add_message(message)
    dialog.add_button("Yes", True, primary=True, default=True)
    dialog.add_button("No", False)
    dialog.add_button("Cancel", None, cancel=True)
    return dialog.run()


# --- Text input --------------------------------------------------------------
# Returns the entered text (stripped), or None if cancelled.
def ask_string(parent: tk.Misc, title: str, prompt: str, initial: str = "") -> Optional[str]:
    dialog = _Dialog(parent, title, "input")
    dialog.add_message(prompt)
    var = tk.StringVar(value=initial)
    entry = ttk.Entry(dialog.body, textvariable=var, style="Field.TEntry", font=(theme.FONT_FAMILY, 11))
    entry.pack(fill="x", pady=(4, 0))
    entry.select_range(0, "end")
    dialog.add_button("OK", lambda: var.get().strip(), primary=True, default=True)
    dialog.add_button("Cancel", None, cancel=True)
    return dialog.run(focus=entry)


# Returns the entered text (which may be empty), or None if cancelled.
def prompt_text(parent: tk.Misc, title: str, prompt: str, initial: str = "") -> Optional[str]:
    dialog = _Dialog(parent, title, "input")
    dialog.add_message(prompt)
    border = tk.Frame(dialog.body, bg=theme.BORDER)
    border.pack(fill="both", expand=True, pady=(4, 0))
    text = tk.Text(border, bg=theme.FIELD, fg=theme.TEXT, insertbackground=theme.TEXT, borderwidth=0, highlightthickness=0,
                   font=(theme.FONT_FAMILY, 11), wrap="word", height=7, width=56, padx=8, pady=6)
    text.pack(fill="both", expand=True, padx=1, pady=1)
    if initial:
        text.insert("1.0", initial)
    # No Return binding here: Return is a newline in a multi-line box.
    dialog.add_button("OK", lambda: text.get("1.0", "end-1c").strip(), primary=True)
    dialog.add_button("Cancel", None, cancel=True)
    return dialog.run(focus=text)


# --- List pickers ---------------------------------------------------------------
def _make_listbox(parent: tk.Widget, options: list[str], *, selectmode: str) -> tk.Listbox:
    border = tk.Frame(parent, bg=theme.BORDER)
    border.pack(fill="both", expand=True, pady=(4, 0))
    listbox = tk.Listbox(
        border,
        bg=theme.FIELD,
        fg=theme.TEXT,
        selectbackground=theme.RED,
        selectforeground="#ffffff",
        highlightthickness=0,
        borderwidth=0,
        activestyle="none",
        font=(theme.FONT_FAMILY, 11),
        height=min(12, max(4, len(options))),
        selectmode=selectmode,
        exportselection=False,
    )
    scrollbar = ttk.Scrollbar(border, orient="vertical", command=listbox.yview, style="Vertical.TScrollbar")
    listbox.configure(yscrollcommand=scrollbar.set)
    listbox.pack(side="left", fill="both", expand=True, padx=1, pady=1)
    if len(options) > 12:
        scrollbar.pack(side="right", fill="y")
    for option in options:
        listbox.insert("end", f"  {option}")
    return listbox


def _make_preview(parent: tk.Widget) -> tk.Label:
    label = tk.Label(parent, text="", bg=theme.BG, fg=theme.MUTED, font=(theme.FONT_FAMILY, 10), justify="left", anchor="w",
                     wraplength=TEXT_WRAP)
    label.pack(anchor="w", fill="x", pady=(10, 0))
    return label


# A modal single-selection list picker (with an optional description preview
# under the list). Returns the chosen item, or None if cancelled.
def choose_from_list(
    parent: tk.Misc,
    title: str,
    prompt: str,
    options: list[str],
    current: Optional[str] = None,
    descriptions: Optional[dict[str, str]] = None,
    confirm_label: str = "Load",
) -> Optional[str]:
    dialog = _Dialog(parent, title, "input")
    dialog.add_message(prompt)
    listbox = _make_listbox(dialog.body, options, selectmode="browse")
    preview = _make_preview(dialog.body) if descriptions is not None else None

    def update_preview(_event: Optional[tk.Event] = None) -> None:
        if preview is not None:
            selection = listbox.curselection()
            preview.configure(text=descriptions.get(options[selection[0]], "") if selection else "")

    index = options.index(current) if current in options else 0
    listbox.selection_set(index)
    listbox.see(index)
    update_preview()
    listbox.bind("<<ListboxSelect>>", update_preview)

    def chosen() -> Optional[str]:
        selection = listbox.curselection()
        return options[selection[0]] if selection else None

    dialog.add_button(confirm_label, chosen, primary=True, default=True)
    dialog.add_button("Cancel", None, cancel=True)
    listbox.bind("<Double-Button-1>", lambda _e: dialog.choose(chosen))
    return dialog.run(focus=listbox)


# A modal multi-selection list picker. Returns the chosen items, or None if
# cancelled / nothing was chosen.
def choose_multiple_from_list(
    parent: tk.Misc,
    title: str,
    prompt: str,
    options: list[str],
    descriptions: Optional[dict[str, str]] = None,
    confirm_label: str = "Delete",
) -> Optional[list[str]]:
    dialog = _Dialog(parent, title, "input")
    dialog.add_message(prompt)
    listbox = _make_listbox(dialog.body, options, selectmode="extended")
    preview = _make_preview(dialog.body) if descriptions is not None else None

    def update_preview(_event: Optional[tk.Event] = None) -> None:
        if preview is None:
            return
        selection = listbox.curselection()
        if len(selection) == 1:
            preview.configure(text=descriptions.get(options[selection[0]], ""))
        else:
            preview.configure(text=f"{len(selection)} selected." if selection else "")

    update_preview()
    listbox.bind("<<ListboxSelect>>", update_preview)

    def chosen() -> Optional[list[str]]:
        selection = listbox.curselection()
        return [options[i] for i in selection] if selection else None

    dialog.add_button(confirm_label, chosen, primary=True)
    dialog.add_button("Cancel", None, cancel=True)
    return dialog.run(focus=listbox)
