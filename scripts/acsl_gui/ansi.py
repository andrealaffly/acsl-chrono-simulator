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
# File:        ansi.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Renders ANSI SGR (color) escape codes -- the ones src/sim-bridge and
#     include/sim-utilities/sim-messages.hpp print with (COLOR_RED,
#     COLOR_GREEN, COLOR_BLUE, COLOR_ORANGE, the inline bright-cyan value
#     color, bold, 256-color codes, ...) -- into tk.Text tags, instead of
#     the raw "\x1b[38;5;214m" escape bytes showing up as garbage text in
#     the Simulator page's console.
#
#     Supports the general SGR subset those sources actually use: reset (0),
#     bold (1/22), the standard 30-37/40-47 and bright 90-97/100-107
#     foreground/background colors, and 256-color 38;5;N / 48;5;N. Anything
#     else (underline, blink, ...) is parsed (so it doesn't leak into the
#     visible text) but not rendered.
###############################################################################

from __future__ import annotations

import re
import tkinter as tk
from dataclasses import dataclass, replace
from typing import Optional

_SGR_RE = re.compile(r"\x1b\[([0-9;]*)m")

# Approximate xterm default 16-color palette.
_STANDARD_RGB = [
    (0, 0, 0), (205, 0, 0), (0, 205, 0), (205, 205, 0),
    (0, 0, 238), (205, 0, 205), (0, 205, 205), (229, 229, 229),
]
_BRIGHT_RGB = [
    (127, 127, 127), (255, 0, 0), (0, 255, 0), (255, 255, 0),
    (92, 92, 255), (255, 0, 255), (0, 255, 255), (255, 255, 255),
]


def _hex(rgb: tuple[int, int, int]) -> str:
    return "#{:02x}{:02x}{:02x}".format(*rgb)


def _xterm256_to_hex(n: int) -> str:
    if n < 8:
        return _hex(_STANDARD_RGB[n])
    if n < 16:
        return _hex(_BRIGHT_RGB[n - 8])
    if n < 232:
        n -= 16
        r, g, b = n // 36, (n % 36) // 6, n % 6
        level = lambda v: 0 if v == 0 else 55 + 40 * v
        return _hex((level(r), level(g), level(b)))
    gray = 8 + (n - 232) * 10
    return _hex((gray, gray, gray))


@dataclass(frozen=True)
class _Style:
    fg: Optional[tuple[str, int]] = None  # ("std" | "bright" | "256", index)
    bg: Optional[tuple[str, int]] = None
    bold: bool = False

    def resolve_fg(self) -> Optional[str]:
        return _resolve(self.fg, self.bold)

    def resolve_bg(self) -> Optional[str]:
        return _resolve(self.bg, False)


def _resolve(spec: Optional[tuple[str, int]], bold: bool) -> Optional[str]:
    if spec is None:
        return None
    kind, index = spec
    if kind == "256":
        return _xterm256_to_hex(index)
    if kind == "std":
        # Conventional terminal behavior: a bold standard color renders as
        # its bright variant (this is how COLOR_ORANGE + the inline "1;36"
        # bold-cyan value color are meant to look).
        return _hex(_BRIGHT_RGB[index] if bold else _STANDARD_RGB[index])
    if kind == "bright":
        return _hex(_BRIGHT_RGB[index])
    return None


def _apply_params(params: list[int], style: _Style) -> _Style:
    i = 0
    while i < len(params):
        code = params[i]
        if code == 0:
            style = _Style()
        elif code == 1:
            style = replace(style, bold=True)
        elif code == 22:
            style = replace(style, bold=False)
        elif code == 38 and i + 2 < len(params) and params[i + 1] == 5:
            style = replace(style, fg=("256", params[i + 2]))
            i += 2
        elif code == 48 and i + 2 < len(params) and params[i + 1] == 5:
            style = replace(style, bg=("256", params[i + 2]))
            i += 2
        elif 30 <= code <= 37:
            style = replace(style, fg=("std", code - 30))
        elif code == 39:
            style = replace(style, fg=None)
        elif 90 <= code <= 97:
            style = replace(style, fg=("bright", code - 90))
        elif 40 <= code <= 47:
            style = replace(style, bg=("std", code - 40))
        elif code == 49:
            style = replace(style, bg=None)
        elif 100 <= code <= 107:
            style = replace(style, bg=("bright", code - 100))
        # Other SGR codes (underline, blink, ...) are consumed so they don't
        # leak into the visible text, but have no Tk rendering equivalent.
        i += 1
    return style


class AnsiConsole:
    """Feeds raw subprocess output (which may contain ANSI SGR color codes)
    into a tk.Text widget, applying color/bold as Tk tags instead of
    printing the escape bytes. Style state persists across calls to write(),
    since a single SGR code can apply to text that arrives in a later
    chunk."""

    def __init__(self, text_widget: tk.Text) -> None:
        self._text = text_widget
        self._style = _Style()
        self._tag_for_style: dict[tuple[Optional[str], Optional[str]], str] = {}

    def write(self, chunk: str) -> None:
        pos = 0
        for match in _SGR_RE.finditer(chunk):
            literal = chunk[pos : match.start()]
            if literal:
                self._insert(literal)
            raw_params = match.group(1)
            params = [int(p) for p in raw_params.split(";") if p != ""] if raw_params else [0]
            self._style = _apply_params(params, self._style)
            pos = match.end()
        remainder = chunk[pos:]
        if remainder:
            self._insert(remainder)

    def reset(self) -> None:
        """Clear accumulated color state -- call this when starting a fresh
        process so a prior run's unterminated color can't bleed into it."""
        self._style = _Style()

    def _insert(self, text: str) -> None:
        tag = self._tag_for(self._style)
        if tag:
            self._text.insert("end", text, tag)
        else:
            self._text.insert("end", text)

    def _tag_for(self, style: _Style) -> Optional[str]:
        fg = style.resolve_fg()
        bg = style.resolve_bg()
        if fg is None and bg is None:
            return None
        key = (fg, bg)
        tag = self._tag_for_style.get(key)
        if tag is None:
            tag = f"ansi{len(self._tag_for_style)}"
            options = {}
            if fg:
                options["foreground"] = fg
            if bg:
                options["background"] = bg
            self._text.tag_configure(tag, **options)
            self._tag_for_style[key] = tag
        return tag
