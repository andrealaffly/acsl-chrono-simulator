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
# File:        settings.py
# Authors:     Giri M. Kumar
# Date:        September 16, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     The Settings page: a tabbed editor (one tab per config/*.yaml file) for
#     the simulator's configuration. Forms are generated directly from each
#     file's parsed structure (see yaml_form.py) rather than hand-coded field
#     by field, so a key added to a config file later appears in the GUI
#     automatically, rendered as:
#
#         MAJOR HEADING            (a top-level yaml section, e.g. "mode:")
#           SUB HEADING             (a nested section, e.g. "controller: tailsitter:")
#           Option                  (a leaf key; numbers/strings sit inline
#             (description)          with their label, booleans are checkboxes)
#
#     The UAV platform is a single dropdown (sim-config.yaml's platform.* are
#     inherently one-of-four), and picking one filters the Controller section
#     down to just that platform's controllers -- so the exclusivity the
#     config's own comments call for ("only one controller can be active at
#     any given time", "only one platform...") is structural rather than
#     something the user has to manage by hand. The same "only one at a time"
#     comments still drive the remaining exclusive-checkbox groups (collision
#     system, trajectory module).
#
#     Two save paths:
#       - Each tab's own "Save <file>" button writes that file in place.
#       - The top "Save All" button asks for a name and writes every config
#         file into a new config/<name>/ folder, leaving the active
#         config/*.yaml files untouched -- a way to snapshot a configuration
#         set without overwriting what's currently live.
###############################################################################

from __future__ import annotations

import shutil
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk
from typing import Callable, Optional

from .. import dialogs, theme
from ..yaml_form import Document, Field, Section, ValueType, humanize
from .base import Page

CONFIG_FILES = ["sim-config.yaml", "phy-config.yaml", "vis-config.yaml"]

# The mission living directly in config/ (as opposed to a named config/<name>/
# folder saved via Save Mission).
DEFAULT_MISSION = "Default"

# Written alongside a mission's config files, holding the free-text
# description entered when the mission was saved.
MISSION_DESCRIPTION_FILENAME = "mission.txt"

TAB_LABELS = {
    "sim-config.yaml": "Simulator",
    "phy-config.yaml": "Physics",
    "vis-config.yaml": "Visualization",
}

# Explicit top-level section order per file. Sections present in the file but
# missing from this list are appended afterwards in their natural file order,
# so a new section added later still shows up without a code change.
SECTION_ORDER: dict[str, list[str]] = {
    "sim-config.yaml": [
        "mode",
        "debug",
        "platform",
        "controller",
        "aerodynamics",
        "motor_disturbances",
        "environment",
        "interp",
        "poly",
    ],
}

# Groups of dotted field paths that a config file's own comments call out as
# mutually exclusive ("Only one ... at a time"), rendered as checkboxes where
# checking one clears the rest. (UAV platform and controller selection are no
# longer here -- they're a dropdown plus a platform-filtered list instead;
# see _render_platform_section / _render_controller_section.)
EXCLUSIVE_GROUPS: dict[str, list] = {
    "sim-config.yaml": [
        ["interp.enabled", "poly.enabled"],
    ],
    "phy-config.yaml": [
        ["collision.BULLET", "collision.MULTICORE"],
    ],
    "vis-config.yaml": [],
}


def _list_stems(directory: Path) -> list[str]:
    if not directory.is_dir():
        return []
    return sorted(p.stem for p in directory.iterdir() if p.is_file() and not p.name.startswith("."))


# Per-file, per-path overrides that render a field as a dropdown of names
# scrubbed from a directory instead of a free-text entry.
FIELD_OVERRIDES: dict[str, dict[str, Callable[[Path], list[str]]]] = {
    "sim-config.yaml": {
        "poly.filename": lambda repo_dir: _list_stems(repo_dir / "chrono-assets" / "trajectories" / "minjerkpoly"),
        "interp.filename": lambda repo_dir: _list_stems(repo_dir / "chrono-assets" / "trajectories" / "interpolation"),
    }
}


class _ExclusiveGroup:
    """When one member BooleanVar is checked, unchecks the rest of the group."""

    def __init__(self) -> None:
        self._vars: list[tk.BooleanVar] = []

    def add(self, var: tk.BooleanVar) -> None:
        self._vars.append(var)
        var.trace_add("write", lambda *_args, v=var: self._on_change(v))

    def _on_change(self, changed: tk.BooleanVar) -> None:
        if not changed.get():
            return
        for other in self._vars:
            if other is not changed and other.get():
                other.set(False)


class _ScrollableFrame(ttk.Frame):
    """A vertically scrollable container, styled to match the dark theme."""

    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent, style="App.TFrame")
        self._canvas = tk.Canvas(self, bg=theme.BG, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self._canvas.yview, style="Vertical.TScrollbar")
        self._canvas.configure(yscrollcommand=scrollbar.set)
        self._canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.body = ttk.Frame(self._canvas, style="App.TFrame", padding=(4, 0, 4, 0))
        self._window = self._canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind("<Configure>", lambda _e: self._canvas.configure(scrollregion=self._canvas.bbox("all")))
        self._canvas.bind("<Configure>", lambda e: self._canvas.itemconfigure(self._window, width=e.width))
        self._canvas.bind("<Enter>", lambda _e: self._bind_wheel())
        self._canvas.bind("<Leave>", lambda _e: self._unbind_wheel())

    def _bind_wheel(self) -> None:
        self._canvas.bind_all("<MouseWheel>", self._on_wheel)
        self._canvas.bind_all("<Button-4>", self._on_wheel)
        self._canvas.bind_all("<Button-5>", self._on_wheel)

    def _unbind_wheel(self) -> None:
        self._canvas.unbind_all("<MouseWheel>")
        self._canvas.unbind_all("<Button-4>")
        self._canvas.unbind_all("<Button-5>")

    def _on_wheel(self, event: tk.Event) -> None:
        if getattr(event, "num", None) == 4:
            self._canvas.yview_scroll(-3, "units")
        elif getattr(event, "num", None) == 5:
            self._canvas.yview_scroll(3, "units")
        else:
            self._canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")


class _FileForm:
    """Holds one config file's parsed Document plus its live tk.Variables."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.document = Document(path)
        self.vars: dict[str, tk.Variable] = {}
        self.entry_widgets: dict[str, ttk.Entry] = {}
        self.dirty = False
        # Only meaningful for sim-config.yaml's platform/controller sections.
        self.platform_var: Optional[tk.StringVar] = None
        self.controller_section: Optional[Section] = None
        self.controller_container: Optional[tk.Widget] = None


class SettingsPage(Page):
    key = "settings"
    label = "Settings"

    def __init__(self, shell, repo_dir: Path) -> None:
        super().__init__(shell)
        self.repo_dir = repo_dir
        self.config_dir = repo_dir / "config"
        self.forms: dict[str, _FileForm] = {}
        self._notebook: Optional[ttk.Notebook] = None
        self._tab_ids: dict[str, str] = {}
        self.current_mission = DEFAULT_MISSION
        self._mission_label: Optional[ttk.Label] = None

    # --- Page interface ---------------------------------------------------
    def build(self, parent: ttk.Frame) -> tk.Widget:
        self._ensure_default_mission_snapshot()

        root = ttk.Frame(parent, style="App.TFrame")

        toolbar = ttk.Frame(root, style="App.TFrame", padding=(0, 0, 0, 10))
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="SETTINGS", style="Eyebrow.TLabel").pack(side="left")
        self._mission_label = ttk.Label(toolbar, text="", style="Body.TLabel")
        self._mission_label.pack(side="left", padx=(14, 0))
        ttk.Button(toolbar, text="Save Mission", style="Primary.TButton", command=self._save_mission).pack(side="right")
        ttk.Button(toolbar, text="Load Mission", style="Secondary.TButton", command=self._load_mission).pack(side="right", padx=(0, 8))
        ttk.Button(toolbar, text="Delete Missions", style="Secondary.TButton", command=self._delete_missions).pack(side="right", padx=(0, 8))

        self._notebook = ttk.Notebook(root, style="TNotebook")
        self._notebook.pack(fill="both", expand=True)

        for filename in CONFIG_FILES:
            self._build_file_tab(filename)
        self._update_mission_label()

        return root

    def has_unsaved_changes(self) -> bool:
        return any(form.dirty for form in self.forms.values())

    # --- Missions -------------------------------------------------------
    # config/ itself is always "the active configuration" -- the one thing
    # build/acsl_sim actually reads. A mission (including "Default") is a
    # named snapshot under config/<name>/. Loading a mission copies its
    # files over the active config/*.yaml; saving a mission writes your
    # current edits to config/*.yaml and then copies that same result into
    # config/<name>/.
    def _mission_dir(self, name: str) -> Path:
        return self.config_dir / name

    def _ensure_default_mission_snapshot(self) -> None:
        """The first time the GUI runs against a config/ with no saved
        missions yet, snapshot whatever is there as the protected 'Default'
        mission, so there's always a baseline to load back to."""
        default_dir = self._mission_dir(DEFAULT_MISSION)
        if default_dir.exists():
            return
        if not all((self.config_dir / f).is_file() for f in CONFIG_FILES):
            return
        default_dir.mkdir(parents=True, exist_ok=True)
        for filename in CONFIG_FILES:
            shutil.copy2(self.config_dir / filename, default_dir / filename)
        (default_dir / MISSION_DESCRIPTION_FILENAME).write_text("The original baseline configuration.\n", encoding="utf-8")

    def _list_missions(self) -> list[str]:
        names = []
        if self.config_dir.is_dir():
            for entry in sorted(self.config_dir.iterdir()):
                if entry.is_dir() and all((entry / f).is_file() for f in CONFIG_FILES):
                    names.append(entry.name)
        if DEFAULT_MISSION in names:
            names.remove(DEFAULT_MISSION)
            names.insert(0, DEFAULT_MISSION)
        return names

    def _read_mission_description_file(self, name: str) -> str:
        description_path = self._mission_dir(name) / MISSION_DESCRIPTION_FILENAME
        if description_path.is_file():
            return description_path.read_text(encoding="utf-8").strip()
        return ""

    def _mission_description(self, name: str) -> str:
        text = self._read_mission_description_file(name)
        return text if text else "(No description provided.)"

    def _update_mission_label(self) -> None:
        if self._mission_label is not None:
            self._mission_label.configure(text=f"Mission: {self.current_mission}")

    def _load_mission(self) -> None:
        save_choice = messagebox.askyesnocancel(
            "Save current configuration?",
            "Save the current configuration as a mission before loading a different one?",
            parent=self.shell.root,
        )
        if save_choice is None:
            return
        if save_choice and not self._save_mission():
            return

        names = self._list_missions()
        if not names:
            messagebox.showinfo("No missions saved", "There are no saved missions yet. Use Save Mission to create one.", parent=self.shell.root)
            return
        descriptions = {name: self._mission_description(name) for name in names}
        choice = dialogs.choose_from_list(
            self.shell.root,
            "Load Mission",
            "Select a mission to load. Its files will overwrite the active configuration in config/.",
            names,
            current=self.current_mission,
            descriptions=descriptions,
        )
        if choice is None:
            return
        self._apply_mission(choice)

    def _delete_missions(self) -> None:
        names = [name for name in self._list_missions() if name != DEFAULT_MISSION]
        if not names:
            messagebox.showinfo("No missions to delete", "There are no saved missions (other than Default) to delete.", parent=self.shell.root)
            return
        descriptions = {name: self._mission_description(name) for name in names}
        chosen = dialogs.choose_multiple_from_list(
            self.shell.root,
            "Delete Missions",
            "Select the missions to permanently delete. 'Default' cannot be deleted here.",
            names,
            descriptions=descriptions,
        )
        if not chosen:
            return
        if not messagebox.askyesno(
            "Delete missions",
            f"Permanently delete {len(chosen)} mission(s): {', '.join(chosen)}?\n\nThis cannot be undone.",
            parent=self.shell.root,
        ):
            return

        deleted: list[str] = []
        for name in chosen:
            try:
                shutil.rmtree(self._mission_dir(name))
                deleted.append(name)
            except OSError as error:
                messagebox.showerror("Delete failed", f"Could not delete '{name}': {error}", parent=self.shell.root)

        if self.current_mission in deleted:
            self.current_mission = "(unknown)"
            self._update_mission_label()
        if deleted:
            self.shell.set_status(f"Deleted {len(deleted)} mission(s): {', '.join(deleted)}.")

    def _apply_mission(self, name: str) -> None:
        mission_dir = self._mission_dir(name)
        for filename in CONFIG_FILES:
            shutil.copy2(mission_dir / filename, self.config_dir / filename)

        for tab_id in self._tab_ids.values():
            self._notebook.forget(tab_id)
        self.forms.clear()
        self._tab_ids.clear()
        self.current_mission = name
        for filename in CONFIG_FILES:
            self._build_file_tab(filename)
        self._update_mission_label()
        self.shell.set_status(f"Loaded mission '{name}' into config/.")

    # --- Tab construction ---------------------------------------------------
    def _build_file_tab(self, filename: str) -> None:
        path = self.config_dir / filename
        form = _FileForm(path)
        self.forms[filename] = form

        scroller = _ScrollableFrame(self._notebook)
        self._notebook.add(scroller, text=TAB_LABELS.get(filename, filename))
        self._tab_ids[filename] = str(scroller)

        groups = self._build_exclusive_groups(filename, form.document)

        sections = [c for c in form.document.root.children if isinstance(c, Section)]
        order = SECTION_ORDER.get(filename, [])
        ordered_keys = [k for k in order if any(s.key == k for s in sections)]
        ordered_keys += [s.key for s in sections if s.key not in order]
        sections_by_key = {s.key: s for s in sections}

        for key in ordered_keys:
            section = sections_by_key[key]
            if filename == "sim-config.yaml" and key == "platform":
                self._render_platform_section(scroller.body, section, form, filename)
            elif filename == "sim-config.yaml" and key == "controller":
                self._render_controller_section(scroller.body, section, form)
            else:
                self._render_node(scroller.body, section, filename, form, groups, depth=0)

        actions = ttk.Frame(scroller.body, style="Section.TFrame", padding=(4, 16, 0, 30))
        actions.pack(fill="x")
        ttk.Button(actions, text=f"Save {filename}", style="Secondary.TButton", command=lambda f=filename: self._save_file(f)).pack(side="left")

    def _build_exclusive_groups(self, filename: str, document: Document) -> dict[str, _ExclusiveGroup]:
        mapping: dict[str, _ExclusiveGroup] = {}
        for spec in EXCLUSIVE_GROUPS.get(filename, []):
            group = _ExclusiveGroup()
            paths = [p for p in spec if p in document.fields]
            for path in paths:
                mapping[path] = group
        return mapping

    # --- Platform (dropdown) + Controller (platform-filtered) --------------
    # UAV platforms aren't a fixed list -- whatever boolean leaves exist
    # under sim-config.yaml's "platform:" section right now are the choices,
    # so a platform (and a matching "controller: <key>:" block) can be added
    # to the yaml later without touching this code. Display labels come from
    # yaml_form.humanize(), never a hand-maintained name table.
    def _platform_keys(self, document: Document) -> list[str]:
        platform_section = self._find_child_section(document.root, "platform")
        if platform_section is None:
            return []
        return [child.key for child in platform_section.children if isinstance(child, Field) and child.value_type is ValueType.BOOL]

    def _current_platform_key(self, document: Document) -> str:
        keys = self._platform_keys(document)
        for key in keys:
            field = document.fields.get(f"platform.{key}")
            if field is not None and field.display_value():
                return key
        return keys[0] if keys else ""

    def _platform_key_from_label(self, document: Document, label: str) -> str:
        keys = self._platform_keys(document)
        for key in keys:
            if humanize(key) == label:
                return key
        return keys[0] if keys else ""

    def _find_child_section(self, section: Section, key: str) -> Optional[Section]:
        for child in section.children:
            if isinstance(child, Section) and child.key == key:
                return child
        return None

    def _render_platform_section(self, parent: tk.Widget, section: Section, form: _FileForm, filename: str) -> None:
        self._render_major_heading(parent, section)
        row = ttk.Frame(parent, style="Section.TFrame", padding=(4, 0, 0, 18))
        row.pack(fill="x")
        ttk.Label(row, text="UAV Platform", style="OptionLabel.TLabel").pack(side="left")

        platform_keys = self._platform_keys(form.document)
        current_key = self._current_platform_key(form.document)
        form.platform_var = tk.StringVar(value=humanize(current_key) if current_key else "")
        combo = ttk.Combobox(
            row,
            textvariable=form.platform_var,
            values=[humanize(k) for k in platform_keys],
            state="readonly",
            style="Field.TCombobox",
            width=22,
        )
        combo.pack(side="left", padx=(10, 0))
        combo.bind("<<ComboboxSelected>>", lambda _e, f=form, fn=filename: self._on_platform_selected(f, fn))

    def _on_platform_selected(self, form: _FileForm, filename: str) -> None:
        key = self._platform_key_from_label(form.document, form.platform_var.get())
        for platform_key in self._platform_keys(form.document):
            form.document.set_bool(f"platform.{platform_key}", platform_key == key)
        # The platform changed, so any previously-active controller is for
        # the wrong platform now -- clear every controller leaf and let the
        # user pick a fresh one that matches the newly selected platform.
        for controller_field in form.document.all_fields_under("controller"):
            form.document.set_bool(controller_field.path, False)
        form.dirty = True
        self._refresh_tab_title(filename)
        self._populate_controller_container(form)

    def _render_controller_section(self, parent: tk.Widget, section: Section, form: _FileForm) -> None:
        self._render_major_heading(parent, section)
        form.controller_section = section
        form.controller_container = ttk.Frame(parent, style="Section.TFrame", padding=(4, 0, 0, 18))
        form.controller_container.pack(fill="x")
        self._populate_controller_container(form)

    def _populate_controller_container(self, form: _FileForm) -> None:
        assert form.controller_container is not None and form.controller_section is not None
        for path in [p for p in form.vars if p.startswith("controller.")]:
            form.vars.pop(path, None)
            form.entry_widgets.pop(path, None)
        for widget in list(form.controller_container.winfo_children()):
            widget.destroy()

        key = self._platform_key_from_label(form.document, form.platform_var.get())
        platform_section = self._find_child_section(form.controller_section, key)
        ttk.Label(
            form.controller_container,
            text=f"{humanize(key)} controllers" if key else "Controllers",
            style="SubHeading.TLabel",
        ).pack(anchor="w", pady=(0, 6))

        if platform_section is None or not platform_section.children:
            ttk.Label(form.controller_container, text="No controllers defined for this platform.", style="OptionHelp.TLabel").pack(anchor="w")
            return

        group = _ExclusiveGroup()
        for child in platform_section.children:
            if isinstance(child, Field):
                local_groups = {child.path: group} if child.value_type is ValueType.BOOL else {}
                self._render_field(form.controller_container, child, "sim-config.yaml", form, local_groups)

    # --- Generic recursive rendering (everything except platform/controller) -
    def _render_major_heading(self, parent: tk.Widget, section: Section) -> None:
        ttk.Label(parent, text=section.title, style="MajorHeading.TLabel").pack(anchor="w", pady=(18, 0))
        if section.help:
            ttk.Label(parent, text=section.help, style="MajorHeadingNote.TLabel", wraplength=860, justify="left").pack(anchor="w", pady=(3, 0))
        divider = tk.Frame(parent, bg=theme.BORDER, height=1)
        divider.pack(fill="x", pady=(9, 10))

    def _render_node(self, parent: tk.Widget, node, filename: str, form: _FileForm, groups: dict, depth: int) -> None:
        if isinstance(node, Field):
            self._render_field(parent, node, filename, form, groups)
        elif isinstance(node, Section):
            body = self._render_section_heading(parent, node, depth)
            for child in node.children:
                self._render_node(body, child, filename, form, groups, depth + 1)

    def _render_section_heading(self, parent: tk.Widget, section: Section, depth: int) -> tk.Widget:
        indent = 4 + depth * 18
        if depth == 0:
            self._render_major_heading(parent, section)
        else:
            ttk.Label(parent, text=section.title, style="SubHeading.TLabel").pack(anchor="w", padx=(indent, 0), pady=(12, 2))
            if section.help:
                ttk.Label(parent, text=section.help, style="SubHeadingNote.TLabel", wraplength=820, justify="left").pack(anchor="w", padx=(indent, 0), pady=(0, 4))
        body = ttk.Frame(parent, style="Section.TFrame", padding=(indent, 0, 0, 0))
        body.pack(fill="x")
        return body

    def _render_field(self, parent: tk.Widget, field: Field, filename: str, form: _FileForm, groups: dict) -> None:
        row = ttk.Frame(parent, style="Section.TFrame", padding=(0, 5, 0, 5))
        row.pack(fill="x")

        label_text = humanize(field.key)

        if field.value_type is ValueType.BOOL:
            var = tk.BooleanVar(value=field.display_value())
            ttk.Checkbutton(row, text=label_text, variable=var, style="Choice.TCheckbutton").pack(anchor="w")
            if field.path in groups:
                groups[field.path].add(var)
            var.trace_add("write", lambda *_a, f=filename, p=field.path: self._on_field_changed(f, p))
            help_padx = (26, 0)
        else:
            inline = ttk.Frame(row, style="Section.TFrame")
            inline.pack(anchor="w", fill="x")
            ttk.Label(inline, text=label_text, style="OptionLabel.TLabel").pack(side="left")

            initial = field.value_text.strip() if field.value_type in (ValueType.INT, ValueType.FLOAT) else str(field.display_value())
            var = tk.StringVar(value=initial)
            override = FIELD_OVERRIDES.get(filename, {}).get(field.path)

            if override is not None:
                entry: ttk.Widget = ttk.Combobox(inline, textvariable=var, values=override(self.repo_dir), state="readonly", style="Field.TCombobox", width=28)
            else:
                width = 10 if field.value_type in (ValueType.INT, ValueType.FLOAT) else 26
                entry = ttk.Entry(inline, textvariable=var, style="Field.TEntry", width=width)
            entry.pack(side="left", padx=(10, 0))
            form.entry_widgets[field.path] = entry

            var.trace_add(
                "write",
                lambda *_a, f=filename, p=field.path, v=var, t=field.value_type, e=entry: self._on_entry_changed(f, p, v, t, e),
            )
            help_padx = (0, 0)

        if field.comment:
            ttk.Label(row, text=field.comment, style="OptionHelp.TLabel", wraplength=780, justify="left").pack(anchor="w", padx=help_padx, pady=(3, 0))

        form.vars[field.path] = var

    # --- Change tracking / validation --------------------------------------
    def _on_field_changed(self, filename: str, path: str) -> None:
        self.forms[filename].dirty = True
        self._refresh_tab_title(filename)

    def _on_entry_changed(self, filename: str, path: str, var: tk.Variable, value_type: ValueType, entry: ttk.Widget) -> None:
        if isinstance(entry, ttk.Entry):
            valid = _is_valid(var.get(), value_type)
            entry.configure(style="Field.TEntry" if valid else "FieldInvalid.TEntry")
        self.forms[filename].dirty = True
        self._refresh_tab_title(filename)

    def _refresh_tab_title(self, filename: str) -> None:
        base = TAB_LABELS.get(filename, filename)
        text = base + (" *" if self.forms[filename].dirty else "")
        self._notebook.tab(self._tab_ids[filename], text=text)

    def _invalid_fields(self, filename: str) -> list[str]:
        form = self.forms[filename]
        return [
            path
            for path, var in form.vars.items()
            if isinstance(var, tk.StringVar) and not _is_valid(var.get(), form.document.fields[path].value_type)
        ]

    def _apply_vars_to_document(self, form: _FileForm) -> None:
        for path, var in form.vars.items():
            field = form.document.fields[path]
            if field.value_type is ValueType.BOOL:
                form.document.set_bool(path, bool(var.get()))
            else:
                form.document.set_raw(path, str(var.get()).strip())

    # --- Save / reload ---------------------------------------------------
    def _save_file(self, filename: str) -> bool:
        invalid = self._invalid_fields(filename)
        if invalid:
            self._show_invalid_fields_error(filename, invalid)
            return False

        form = self.forms[filename]
        self._apply_vars_to_document(form)
        form.document.save()
        form.dirty = False
        self._refresh_tab_title(filename)
        self.shell.set_status(f"Saved {filename}.")
        return True

    def _show_invalid_fields_error(self, filename: str, invalid_paths: list[str]) -> None:
        form = self.forms[filename]
        names = ", ".join(humanize(form.document.fields[p].key) for p in invalid_paths)
        messagebox.showerror("Invalid value", f"Fix the following field(s) in {filename} before saving:\n\n{names}", parent=self.shell.root)

    def _save_mission(self) -> bool:
        """Write current edits to the active config/*.yaml, then copy that
        result into config/<name>/ as a named, described snapshot. Returns
        whether it actually saved (False if the user backed out of any
        step), so callers like _load_mission can tell "saved" from
        "cancelled"."""
        for filename in CONFIG_FILES:
            invalid = self._invalid_fields(filename)
            if invalid:
                self._show_invalid_fields_error(filename, invalid)
                return False

        name = simpledialog.askstring(
            "Save mission",
            "Name this mission. It's saved as a folder under config/ "
            f"(use '{DEFAULT_MISSION}' to update the protected baseline):",
            initialvalue=self.current_mission,
            parent=self.shell.root,
        )
        if name is None:
            return False
        name = name.strip()
        if not name or "/" in name or "\\" in name or name in (".", ".."):
            messagebox.showerror("Invalid name", "Please enter a simple mission name (no slashes).", parent=self.shell.root)
            return False

        description = dialogs.prompt_text(
            self.shell.root,
            "Mission description",
            f"Describe this scenario ('{name}'). This is shown when loading missions later.",
            initial=self._read_mission_description_file(name),
        )
        if description is None:
            return False

        target_dir = self._mission_dir(name)
        if name == DEFAULT_MISSION and target_dir.exists():
            if not messagebox.askyesno(
                "Overwrite protected baseline",
                "This will overwrite the protected 'Default' baseline you can always load back to. Continue?",
                parent=self.shell.root,
            ):
                return False
        elif target_dir.exists():
            if not messagebox.askyesno("Mission exists", f"A mission named '{name}' already exists. Overwrite it?", parent=self.shell.root):
                return False
        target_dir.mkdir(parents=True, exist_ok=True)
        (target_dir / MISSION_DESCRIPTION_FILENAME).write_text(description + "\n" if description else "", encoding="utf-8")

        for filename in CONFIG_FILES:
            form = self.forms[filename]
            self._apply_vars_to_document(form)
            form.document.save()  # -> config/<filename>, the active configuration
            shutil.copy2(form.document.path, target_dir / filename)  # -> config/<name>/<filename>, the snapshot
            form.dirty = False
            self._refresh_tab_title(filename)

        self.current_mission = name
        self._update_mission_label()
        self.shell.set_status(f"Saved mission '{name}' (and updated the active configuration in config/).")
        return True


def _is_valid(text: str, value_type: ValueType) -> bool:
    if value_type is ValueType.INT:
        try:
            int(text.strip())
            return True
        except ValueError:
            return False
    if value_type is ValueType.FLOAT:
        try:
            float(text.strip())
            return True
        except ValueError:
            return False
    return True
