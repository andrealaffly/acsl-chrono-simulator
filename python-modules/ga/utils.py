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
# File:        utils.py
# Authors:     Giri M. Kumar
# Date:        October 26, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Small helpers for nested dictionaries, shared by the whole package.
#
#     Both the GA config (YAML) and a controller's gains file (JSON) are
#     nested dictionaries, and the GA addresses a value inside them with a
#     "dotted path": the keys from the top down, joined by dots. For example
#         "algorithm.population_size"        in ga-config.yaml
#         "TRANSLATIONAL.KP_translational"   in gains_<CTRL>.json
#     get_dotted / set_dotted read and write a value at such a path, and
#     deep_merge lays one config on top of another (used to apply a session
#     file over config/ga-config.yaml).
###############################################################################

from __future__ import annotations

import copy
from typing import Any


def get_dotted(data: Any, path: str) -> Any:
    """Return the value at a dotted path, or None if any key on the way is
    missing (or the value on the way isn't a dictionary).

        get_dotted({"a": {"b": 3}}, "a.b")  ->  3
        get_dotted({"a": {"b": 3}}, "a.c")  ->  None
    """
    for key in path.split("."):
        if not isinstance(data, dict) or key not in data:
            return None
        data = data[key]
    return data


def set_dotted(data: dict, path: str, value: Any) -> None:
    """Set the value at a dotted path, creating any missing parent
    dictionaries on the way.

        d = {}
        set_dotted(d, "a.b", 3)   # d is now {"a": {"b": 3}}
    """
    *parents, last = path.split(".")
    for key in parents:
        data = data.setdefault(key, {})
    data[last] = value


def deep_merge(base: dict, override: dict) -> dict:
    """Return a copy of `base` with `override` laid on top of it.

    Nested dictionaries are merged key by key; anything else (numbers,
    strings, lists) in `override` replaces the value in `base` outright.
    That last point matters for the GA's "tune" list: a session's list
    replaces the default one instead of being appended to it.
    Neither input is modified.
    """
    merged = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged
