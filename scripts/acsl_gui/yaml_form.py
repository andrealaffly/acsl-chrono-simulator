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
# File:        yaml_form.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     A small, comment-preserving reader/writer for the simulator's config
#     YAML files (config/sim-config.yaml, phy-config.yaml, vis-config.yaml).
#
#     This is intentionally not a general-purpose YAML library. It understands
#     exactly the shape these three files use: nested block mappings of
#     booleans/numbers/strings, with descriptive comment blocks above each
#     key. It parses that structure into a Document tree keyed by dotted path
#     (e.g. "controller.tailsitter.mrac_hybrid"), using the comment
#     immediately above a key as that field's help text (and, for section
#     keys, as the section's display title).
#
#     Saving rewrites only the value portion of the specific lines that
#     changed; every other line (comments, blank lines, section headers) is
#     left byte-for-byte untouched.
###############################################################################

from __future__ import annotations

import re
from dataclasses import dataclass, field as dc_field
from enum import Enum, auto
from pathlib import Path
from typing import Optional, Union

_LINE_RE = re.compile(r"^(?P<indent>[ \t]*)(?P<key>[A-Za-z0-9_\-]+):(?P<rest>.*)$")
_DIVIDER_RE = re.compile(r"^[=\-#!\s]*$")
_INT_RE = re.compile(r"^[+-]?\d+$")
_FLOAT_RE = re.compile(r"^[+-]?(\d+\.\d*|\.\d+|\d+)([eE][+-]?\d+)?$")


class ValueType(Enum):
    BOOL = auto()
    INT = auto()
    FLOAT = auto()
    STRING = auto()


def _infer_type(value_text: str) -> tuple[ValueType, bool]:
    """Return (type, was_quoted) for a raw scalar's text as it appears in the file."""
    text = value_text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return ValueType.STRING, True
    if text.lower() in ("true", "false"):
        return ValueType.BOOL, False
    if _INT_RE.match(text):
        return ValueType.INT, False
    if _FLOAT_RE.match(text) and ("." in text or "e" in text.lower()):
        return ValueType.FLOAT, False
    return ValueType.STRING, False


def _split_value_and_comment(rest: str) -> tuple[str, str]:
    """Split the text after 'key:' into (value_text, trailing_comment)."""
    text = rest.strip()
    if not text:
        return "", ""
    if text[0] in "\"'":
        quote = text[0]
        end = text.find(quote, 1)
        if end != -1:
            value_text = text[: end + 1]
            remainder = text[end + 1 :].strip()
            return value_text, remainder
    hash_index = text.find("#")
    if hash_index == -1:
        return text.strip(), ""
    return text[:hash_index].strip(), text[hash_index:].strip()


@dataclass
class Field:
    """A leaf key: value entry (a setting the GUI can display/edit)."""

    key: str
    path: str
    indent: str
    value_text: str
    trailing_comment: str
    comment: str
    line_index: int
    value_type: ValueType
    quoted: bool

    def display_value(self) -> Union[bool, int, float, str]:
        text = self.value_text.strip()
        if self.value_type is ValueType.BOOL:
            return text.lower() == "true"
        if self.value_type is ValueType.INT:
            return int(text)
        if self.value_type is ValueType.FLOAT:
            return float(text)
        if self.quoted and len(text) >= 2:
            return text[1:-1]
        return text


@dataclass
class Section:
    """A mapping key (has nested children, no scalar value of its own)."""

    key: str
    path: str
    title: str
    help: str
    children: list = dc_field(default_factory=list)  # list[Union[Field, Section]]


def humanize(key: str) -> str:
    """Turn a yaml key (snake_case, camelCase, or an ACRONYM) into a label."""
    if key.isupper():
        return key
    spaced = re.sub(r"[_\-]+", " ", key)
    spaced = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", spaced)
    spaced = re.sub(r"(?<=[A-Z])(?=[A-Z][a-z])", " ", spaced)
    return spaced.title()


class Document:
    """A parsed, editable, comment-preserving view of one config YAML file."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.raw_lines: list[str] = path.read_text(encoding="utf-8").splitlines()
        self.fields: dict[str, Field] = {}
        self.root = Section(key="", path="", title="", help="")
        self._parse()

    # --- Parsing ------------------------------------------------------------
    def _parse(self) -> None:
        stack: list[tuple[int, Section]] = [(-1, self.root)]
        pending: list[str] = []

        for index, raw_line in enumerate(self.raw_lines):
            stripped = raw_line.strip()

            if stripped == "":
                pending = []
                continue

            if stripped.startswith("#"):
                text = stripped.lstrip("#").strip()
                if text and not _DIVIDER_RE.match(text):
                    pending.append(text)
                continue

            match = _LINE_RE.match(raw_line)
            if not match:
                pending = []
                continue

            indent_str = match.group("indent")
            key = match.group("key")
            rest = match.group("rest")
            indent_level = len(indent_str.expandtabs(2))

            while stack and stack[-1][0] >= indent_level:
                stack.pop()
            parent = stack[-1][1] if stack else self.root
            path = f"{parent.path}.{key}" if parent.path else key
            comment_lines = pending
            comment_text = " ".join(pending)
            pending = []

            value_text, trailing_comment = _split_value_and_comment(rest)

            if value_text == "":
                # First comment line (usually a short banner title) becomes the
                # section's display title; any remaining lines (e.g. an
                # "!!! IMPORTANT ..." exclusivity note) become its help text.
                section_title = comment_lines[0] if comment_lines else humanize(key)
                section_help = " ".join(comment_lines[1:]) if len(comment_lines) > 1 else ""
                section = Section(key=key, path=path, title=section_title, help=section_help)
                parent.children.append(section)
                stack.append((indent_level, section))
            else:
                value_type, quoted = _infer_type(value_text)
                node = Field(
                    key=key,
                    path=path,
                    indent=indent_str,
                    value_text=value_text,
                    trailing_comment=trailing_comment,
                    comment=comment_text,
                    line_index=index,
                    value_type=value_type,
                    quoted=quoted,
                )
                parent.children.append(node)
                self.fields[path] = node

    # --- Access / mutation --------------------------------------------------
    def all_fields_under(self, prefix: str) -> list[Field]:
        return [f for p, f in self.fields.items() if p == prefix or p.startswith(prefix + ".")]

    def set_bool(self, path: str, value: bool) -> None:
        self._set_value_text(path, "true" if value else "false")

    def set_raw(self, path: str, raw_text: str) -> None:
        """Set a field's value from user-entered text, re-applying quotes if it was a quoted string."""
        field = self.fields[path]
        if field.quoted:
            self._set_value_text(path, f'"{raw_text}"')
        else:
            self._set_value_text(path, raw_text)

    def _set_value_text(self, path: str, new_value_text: str) -> None:
        field = self.fields[path]
        field.value_text = new_value_text
        line = f"{field.indent}{field.key}: {new_value_text}"
        if field.trailing_comment:
            line += f"  {field.trailing_comment}"
        self.raw_lines[field.line_index] = line

    # --- Persistence ----------------------------------------------------
    def render(self) -> str:
        return "\n".join(self.raw_lines) + "\n"

    def save(self) -> None:
        """Write the current (in-memory edited) content to self.path, in place."""
        self.path.write_text(self.render(), encoding="utf-8")

    def save_as(self, target_path: Path) -> None:
        """Write the current (in-memory edited) content to a different path,
        without touching self.path or the file it points to."""
        target_path.write_text(self.render(), encoding="utf-8")
