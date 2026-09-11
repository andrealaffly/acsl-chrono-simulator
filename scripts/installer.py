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
# File:        installer.py
# Authors:     Giri M. Kumar
# Date:        September 11, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Graphical installer for the ACSL UAV Simulator development environment.
#
#     This installer is intended to be run from the repository's scripts/
#     directory with administrator privileges. It validates the host operating
#     system, presents a graphical prerequisite selection workflow, and installs
#     the selected Project Chrono and ACSL simulator dependencies.
#
#     Main functionality includes:
#
#       - Ubuntu environment and administrator-privilege validation.
#       - License presentation and user acceptance workflow.
#       - Optional apt package update and upgrade stage.
#       - Detection, selection, and installation of simulator prerequisites.
#       - VulkanSceneGraph (VSG) installation and CMake package validation.
#       - ROS 2 detection or installation for supported Ubuntu releases.
#       - Optional ROS 2 environment setup in the invoking user's .bashrc.
#       - Detection of an existing chrono-ros-messages workspace build.
#       - User choice to skip an existing ROS workspace build or clean and
#         rebuild its generated build/, install/, and log/ directories.
#       - Project Chrono CMake configuration from libraries/chrono-build.
#       - Project Chrono configuration with ACSL-required modules enabled:
#           CASCADE
#           IRRLICHT
#           MULTICORE
#           OPENGL
#           POSTPROCESS
#           VEHICLE
#           VSG
#       - Release build configuration through CMAKE_BUILD_TYPE=Release.
#       - Final Project Chrono compilation after explicit user confirmation.
#
#     Generated build artifacts are kept out of version control through local
#     .gitignore files. The installer avoids deleting source directories and
#     restricts cleanup operations to known generated build locations.
#
# GitHub:
#     https://github.com/andrealaffly/acsl-chrono-simulator.git
###############################################################################

#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import platform
import pwd
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Callable, Optional

APP_TITLE = "ACSL UAV Simulator"
APP_TAGLINE = "A Project-Chrono Based High Fidelity Simulator for UAVs"
EXPECTED_DIR_NAME = "scripts"
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parent
ASSET_DIR = REPO_DIR / "chrono-assets" / "sim-assets"
LOGO_PATH = ASSET_DIR / "acsl_sim_logo.jpeg"
PROGRESS_GIF_PATH = ASSET_DIR / "progress.gif"
MINIMUM_CMAKE_VERSION = (3, 20, 0)
BUILD_GITIGNORE = "# Generated build output.\n*\n!.gitignore\n"
ROS_BY_UBUNTU = {"22.04": "humble", "24.04": "jazzy"}


class InstallerError(RuntimeError):
    """An expected installer error."""


class ComponentInstallError(InstallerError):
    """Records the failed component and components completed beforehand."""

    def __init__(self, component: str, completed: list[str], cause: Exception) -> None:
        self.component = component
        self.completed = completed.copy()
        self.cause = cause
        super().__init__(f"{component} installation failed.")


def build_jobs() -> int:
    """Use one fewer CPU than the host reports, while retaining at least one job."""
    return max(1, (os.cpu_count() or 1) - 2)


def command_exists(command: str) -> bool:
    return shutil.which(command) is not None


def run(command: list[str], *, cwd: Optional[Path] = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Run a command while preserving stdout/stderr in the launch terminal."""
    return subprocess.run(command, cwd=str(cwd) if cwd else None, check=check, text=True)


def apt_install(packages: list[str]) -> None:
    if packages:
        run(["apt", "install", "-y", *packages])


def ensure_ignored_directory(directory: Path) -> None:
    """Create repository build folders with a local generated-file .gitignore."""
    try:
        directory.resolve().relative_to(REPO_DIR.resolve())
    except ValueError as error:
        raise InstallerError(f"Refusing to create a build directory outside the repository:\n\n{directory}") from error

    directory.mkdir(parents=True, exist_ok=True)
    gitignore = directory / ".gitignore"
    if not gitignore.exists():
        gitignore.write_text(BUILD_GITIGNORE, encoding="utf-8")


def ubuntu_version() -> Optional[str]:
    os_release = Path("/etc/os-release")
    if not os_release.is_file():
        return None
    values: dict[str, str] = {}
    for line in os_release.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value.strip().strip('"')
    return values.get("VERSION_ID") if values.get("ID") == "ubuntu" else None


def is_wsl() -> bool:
    return bool(os.environ.get("WSL_DISTRO_NAME")) or "microsoft" in platform.release().lower()


def ros_distribution() -> Optional[str]:
    return ROS_BY_UBUNTU.get(ubuntu_version())


def ros_is_installed(distribution: str) -> bool:
    return Path(f"/opt/ros/{distribution}/setup.bash").is_file()


def invoking_user_home() -> Path:
    """Use the sudo-invoking user's home, rather than root's /root home."""
    username = os.environ.get("SUDO_USER")
    if username:
        return Path(pwd.getpwnam(username).pw_dir)
    return Path.home()


def cmake_version() -> Optional[tuple[int, int, int]]:
    if not command_exists("cmake"):
        return None
    result = subprocess.run(["cmake", "--version"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False)
    if result.returncode != 0 or not result.stdout:
        return None
    try:
        token = result.stdout.splitlines()[0].rsplit(" ", 1)[-1]
        version = [int(part) for part in token.split(".")]
        version.extend([0] * (3 - len(version)))
        return version[0], version[1], version[2]
    except (IndexError, ValueError):
        return None


def cmake_is_sufficient() -> bool:
    version = cmake_version()
    return version is not None and version >= MINIMUM_CMAKE_VERSION


def validate_environment() -> None:
    if os.geteuid() != 0:
        raise InstallerError("Administrator privileges are required.\n\nRun:\n\nsudo python3 installer.py")
    if Path.cwd().name != EXPECTED_DIR_NAME:
        raise InstallerError(f"Run this installer from a directory named '{EXPECTED_DIR_NAME}'.\n\nCurrent directory: {Path.cwd()}")
    if ubuntu_version() not in ROS_BY_UBUNTU:
        detected = ubuntu_version() or "a non-Ubuntu system"
        raise InstallerError(f"This installer supports Ubuntu 22.04 and Ubuntu 24.04 only.\n\nDetected: {detected}")
    if not command_exists("apt"):
        raise InstallerError("The Ubuntu apt package manager is required.")


def ensure_gui_dependencies() -> None:
    """Install Tkinter before it is imported; never launch a Tk diagnostics window."""
    if importlib.util.find_spec("tkinter") is not None:
        return
    print("[ACSL] Installing required GUI support...", flush=True)
    run(["apt", "update"])
    apt_install(["python3-tk"])
    if importlib.util.find_spec("tkinter") is None:
        raise InstallerError("python3-tk was installed but Tkinter remains unavailable.\n\nOpen a new terminal and run the installer again.")


def read_text_file(path: Path, label: str) -> str:
    if not path.is_file():
        raise InstallerError(f"{label} file was not found:\n\n{path}")
    return path.read_text(encoding="utf-8", errors="replace").replace("\r\n", "\n")


def license_text() -> str:
    return read_text_file(REPO_DIR / "LICENSE", "License")


def pkg_config_exists(package: str) -> bool:
    if not command_exists("pkg-config"):
        return False
    return subprocess.run(["pkg-config", "--exists", package], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False).returncode == 0


def dpkg_package_installed(package: str) -> bool:
    """Exact package-status test; avoids false positives from `dpkg -l | grep`."""
    if not command_exists("dpkg-query"):
        return False
    result = subprocess.run(["dpkg-query", "-W", "-f=${db:Status-Status}", package], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, check=False)
    return result.returncode == 0 and result.stdout.strip() == "installed"


def any_glob_exists(pattern: str) -> bool:
    return any(Path("/").glob(pattern.lstrip("/")))


@dataclass
class InstallItem:
    name: str
    description: str
    check: Callable[[], bool]
    install: Optional[Callable[[], None]]
    category: str
    installed: Optional[bool] = None
    selected: bool = False

    def refresh(self) -> bool:
        self.installed = bool(self.check())
        self.selected = not self.installed
        return self.installed


ensure_gui_dependencies()

import tkinter as tk
from tkinter import scrolledtext, ttk


class InstallerUI:
    """Fixed ACSL GUI and reusable template for all installation work screens."""

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
    RED_PALE = "#fecaca"
    GREEN = "#5eea8b"
    GREEN_DARK = "#13251b"
    GREEN_BORDER = "#28633b"
    AMBER = "#fbbf24"

    WINDOW_WIDTH = 1040
    WINDOW_HEIGHT = 1140
    GIF_INTERVAL_MS = 55
    GIF_MAX_WIDTH = 1140
    GIF_TOP_PADDING = 30
    MEDIA_BOTTOM_PADDING = 14
    GRID_COLUMNS = 2
    PICKER_TILE_WIDTH = 293
    PICKER_TILE_HEIGHT = 61
    PICKER_NAME_MAX_CHARS = 31
    PICKER_DESCRIPTION_MAX_CHARS = 38

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.withdraw()
        self.root.title(f"{APP_TITLE} — Installer")
        self.root.geometry(f"{self.WINDOW_WIDTH}x{self.WINDOW_HEIGHT}")
        self.root.resizable(False, False)
        self.root.configure(bg=self.BG)
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        self._exit_requested = False
        self._logo_image: Optional[tk.PhotoImage] = None
        self._gif_frames: list[tk.PhotoImage] = []
        self._gif_after_id: Optional[str] = None
        self._gif_label: Optional[tk.Label] = None
        self._template_progress: Optional[ttk.Progressbar] = None
        self._template_status: Optional[tk.Label] = None
        self._template_media: Optional[tk.Frame] = None
        self._template_actions: Optional[ttk.Frame] = None
        self._check_cells: dict[str, tk.Label] = {}
        self._setup_styles()
        self._build_window()
        self._center_window()
        self._precache_progress_gif()
        self._show_starting_screen()
        self.root.deiconify()

    def _center_window(self) -> None:
        self.root.update_idletasks()
        x = max(0, (self.root.winfo_screenwidth() - self.WINDOW_WIDTH) // 2)
        y = max(0, (self.root.winfo_screenheight() - self.WINDOW_HEIGHT) // 2)
        self.root.geometry(f"{self.WINDOW_WIDTH}x{self.WINDOW_HEIGHT}+{x}+{y}")

    def _setup_styles(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("App.TFrame", background=self.BG)
        style.configure("Surface.TFrame", background=self.SURFACE)
        style.configure("Header.TLabel", background=self.BG, foreground=self.TEXT, font=("DejaVu Sans", 21, "bold"))
        style.configure("Tagline.TLabel", background=self.BG, foreground=self.RED, font=("DejaVu Sans", 10, "bold"))
        style.configure("Eyebrow.TLabel", background=self.SURFACE, foreground=self.RED, font=("DejaVu Sans", 9, "bold"))
        style.configure("Title.TLabel", background=self.SURFACE, foreground=self.TEXT, font=("DejaVu Sans", 18, "bold"))
        style.configure("Body.TLabel", background=self.SURFACE, foreground=self.MUTED, font=("DejaVu Sans", 10))
        style.configure("Primary.TButton", background=self.RED, foreground="#ffffff", borderwidth=0, font=("DejaVu Sans", 10, "bold"), padding=(18, 10))
        style.map("Primary.TButton", background=[("active", self.RED_ACTIVE), ("pressed", self.RED_DARK)], foreground=[("disabled", "#969090")])
        style.configure("Secondary.TButton", background=self.SURFACE_2, foreground=self.TEXT, borderwidth=0, font=("DejaVu Sans", 10), padding=(18, 10))
        style.map("Secondary.TButton", background=[("active", self.BORDER), ("pressed", self.BORDER)])
        style.configure("Choice.TCheckbutton", background=self.SURFACE, foreground=self.TEXT, font=("DejaVu Sans", 10), padding=4, borderwidth=0, relief="flat")
        style.map("Choice.TCheckbutton", background=[("active", self.SURFACE), ("disabled", self.SURFACE)], foreground=[("active", self.TEXT), ("disabled", self.DIM)])
        style.configure("Red.Horizontal.TProgressbar", troughcolor=self.FIELD, background=self.RED, bordercolor=self.FIELD, lightcolor=self.RED, darkcolor=self.RED)
        style.configure("Green.Horizontal.TProgressbar", troughcolor=self.FIELD, background=self.GREEN, bordercolor=self.FIELD, lightcolor=self.GREEN, darkcolor=self.GREEN)

    def _load_photo(self, path: Path, max_width: int) -> Optional[tk.PhotoImage]:
        if not path.is_file():
            return None
        try:
            image = tk.PhotoImage(file=str(path))
        except tk.TclError:
            return None
        if image.width() > max_width:
            factor = max(1, (image.width() + max_width - 1) // max_width)
            image = image.subsample(factor, factor)
        return image

    def _load_logo(self) -> Optional[tk.PhotoImage]:
        self._logo_image = self._load_photo(LOGO_PATH, max_width=700)
        return self._logo_image

    def _precache_progress_gif(self) -> None:
        if not PROGRESS_GIF_PATH.is_file():
            return
        frames: list[tk.PhotoImage] = []
        index = 0
        while True:
            try:
                frame = tk.PhotoImage(file=str(PROGRESS_GIF_PATH), format=f"gif -index {index}")
            except tk.TclError:
                break
            if frame.width() > self.GIF_MAX_WIDTH:
                factor = max(1, (frame.width() + self.GIF_MAX_WIDTH - 1) // self.GIF_MAX_WIDTH)
                frame = frame.subsample(factor, factor)
            frames.append(frame)
            index += 1
        self._gif_frames = frames

    def _build_window(self) -> None:
        outer = ttk.Frame(self.root, style="App.TFrame", padding=(34, 26, 34, 22))
        outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer, style="App.TFrame")
        header.pack(fill="x", pady=(0, 18))
        logo = self._load_logo()
        if logo is not None:
            tk.Label(header, image=logo, bg=self.BG, borderwidth=0).pack(anchor="center", pady=(0, 10))
        else:
            ttk.Label(header, text=APP_TITLE, style="Header.TLabel").pack(anchor="center")
            ttk.Label(header, text=APP_TAGLINE.upper(), style="Tagline.TLabel").pack(anchor="center", pady=(5, 0))
        self.card = ttk.Frame(outer, style="Surface.TFrame", padding=26)
        self.card.pack(fill="both", expand=True)
        self.status_var = tk.StringVar(value="Starting installer")
        footer = ttk.Frame(outer, style="App.TFrame")
        footer.pack(fill="x", pady=(14, 0))
        tk.Label(footer, textvariable=self.status_var, bg=self.BG, fg=self.MUTED, font=("DejaVu Sans", 9)).pack(side="left")
        tk.Label(footer, text=f"ACSL • Ubuntu {ubuntu_version() or 'unknown'}", bg=self.BG, fg=self.DIM, font=("DejaVu Sans", 9)).pack(side="right")

    def _stop_gif(self) -> None:
        if self._gif_after_id is not None:
            try:
                self.root.after_cancel(self._gif_after_id)
            except tk.TclError:
                pass
        self._gif_after_id = None
        self._gif_label = None

    def _clear_card(self) -> None:
        self._stop_gif()
        self._template_progress = None
        self._template_status = None
        self._template_media = None
        self._template_actions = None
        self._check_cells = {}
        for child in self.card.winfo_children():
            child.destroy()

    def _exit(self) -> None:
        self._exit_requested = True
        self.root.quit()

    def _heading(self, eyebrow: str, title: str, description: str = "") -> None:
        ttk.Label(self.card, text=eyebrow.upper(), style="Eyebrow.TLabel").pack(anchor="w")
        ttk.Label(self.card, text=title, style="Title.TLabel").pack(anchor="w", pady=(6, 0))
        if description:
            body = ttk.Label(self.card, text=description, style="Body.TLabel", justify="left")
            body.pack(anchor="w", fill="x", pady=(8, 18))
            body.bind("<Configure>", lambda event: body.configure(wraplength=max(event.width - 8, 400)))

    def _scroll_text(self, content: str, height: int = 22) -> scrolledtext.ScrolledText:
        text = scrolledtext.ScrolledText(self.card, wrap="word", height=height, font=("DejaVu Sans", 10), background=self.FIELD, foreground=self.TEXT, insertbackground=self.TEXT, selectbackground="#8f2526", relief="flat", borderwidth=0, padx=16, pady=14)
        text.insert("1.0", content)
        text.configure(state="disabled")
        return text

    def _show_starting_screen(self) -> None:
        self._clear_card()
        self.status_var.set("Starting installer")
        self._heading("Starting installer", "Preparing the ACSL installation workflow", "Loading the installer interface and preparing required local resources.")
        tk.Label(self.card, text="STARTING INSTALLER", bg=self.SURFACE, fg=self.RED, font=("DejaVu Sans", 13, "bold"), justify="center", anchor="center").pack(fill="x", pady=(70, 0))
        self.root.update_idletasks()
        self.root.update()

    def begin_work_screen(self, eyebrow: str, title: str, description: str, status: str, determinate: bool = False) -> None:
        self._clear_card()
        self.status_var.set(status)
        self._heading(eyebrow, title, description)
        mode = "determinate" if determinate else "indeterminate"
        self._template_progress = ttk.Progressbar(self.card, style="Green.Horizontal.TProgressbar" if determinate else "Red.Horizontal.TProgressbar", orient="horizontal", mode=mode, maximum=100, value=0)
        self._template_progress.pack(fill="x", pady=(18, 0))
        if not determinate:
            self._template_progress.start(12)
        self._template_status = tk.Label(self.card, text=status, bg=self.SURFACE, fg=self.AMBER, font=("DejaVu Sans", 10, "bold"), justify="center", anchor="center")
        self._template_status.pack(fill="x", pady=(20, 0))
        self._template_media = tk.Frame(self.card, bg=self.SURFACE)
        self._template_media.pack(fill="both", expand=True)
        self._template_actions = ttk.Frame(self.card, style="Surface.TFrame")
        self._template_actions.pack(fill="x", side="bottom", pady=(20, 0))
        self.root.update_idletasks()
        self.root.update()

    def set_work_status(self, status: str, percent: Optional[int] = None) -> None:
        self.status_var.set(status)
        if self._template_status is not None:
            self._template_status.configure(text=status)
        if percent is not None and self._template_progress is not None:
            self._template_progress.configure(value=percent)
        self.root.update_idletasks()
        self.root.update()

    def _animate_gif(self, frame_index: int = 0) -> None:
        if self._gif_label is None or not self._gif_frames:
            return
        try:
            if not self._gif_label.winfo_exists():
                return
            self._gif_label.configure(image=self._gif_frames[frame_index])
        except tk.TclError:
            return
        self._gif_after_id = self.root.after(self.GIF_INTERVAL_MS, self._animate_gif, (frame_index + 1) % len(self._gif_frames))

    def show_template_animation(self) -> None:
        if self._template_media is None:
            return
        for child in self._template_media.winfo_children():
            child.destroy()
        if not self._gif_frames:
            tk.Label(self._template_media, text="Working", bg=self.SURFACE, fg=self.DIM, font=("DejaVu Sans", 11), justify="center", anchor="center").pack(anchor="center", side="bottom", pady=(0, self.MEDIA_BOTTOM_PADDING))
            return
        self._gif_label = tk.Label(self._template_media, image=self._gif_frames[0], bg=self.SURFACE, borderwidth=0)
        self._gif_label.pack(anchor="center", side="bottom", pady=(self.GIF_TOP_PADDING, self.MEDIA_BOTTOM_PADDING))
        self._animate_gif()

    def run_background_task(self, task: Callable[[], None], poll_ms: int = 10) -> None:
        """Run blocking work off the Tk thread so GIF animation remains responsive."""
        completed: Queue[tuple[Optional[BaseException], Optional[object]]] = Queue(maxsize=1)

        def worker() -> None:
            try:
                task()
                completed.put((None, None))
            except BaseException as error:
                completed.put((error, error.__traceback__))

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        while True:
            try:
                error, traceback_obj = completed.get_nowait()
            except Empty:
                try:
                    self.root.update_idletasks()
                    self.root.update()
                except tk.TclError as ui_error:
                    raise InstallerError("The installer window was closed while work was running.") from ui_error
                time.sleep(poll_ms / 1000)
                continue
            if error is not None:
                raise error.with_traceback(traceback_obj)  # type: ignore[arg-type]
            return

    def finish_work_screen(self, message: str, continue_label: str = "Continue") -> bool:
        if self._template_media is None or self._template_actions is None:
            raise InstallerError("The installer screen template has not been initialized.")
        self._stop_gif()
        self.status_var.set("Complete")
        if self._template_progress is not None:
            self._template_progress.stop()
            self._template_progress.configure(style="Green.Horizontal.TProgressbar", mode="determinate", value=100, maximum=100)
        if self._template_status is not None:
            self._template_status.configure(text="Complete", fg=self.GREEN)
        for child in self._template_media.winfo_children():
            child.destroy()
        for child in self._template_actions.winfo_children():
            child.destroy()
        success = tk.Frame(self._template_media, bg=self.GREEN_DARK, highlightbackground=self.GREEN_BORDER, highlightthickness=1)
        success.pack(anchor="center", side="bottom", fill="x", padx=30, pady=(0, self.MEDIA_BOTTOM_PADDING))
        message_label = tk.Label(success, text=message, bg=self.GREEN_DARK, fg="#c8f7d5", font=("DejaVu Sans", 12, "bold"), justify="center", anchor="center", padx=24, pady=22)
        message_label.pack(fill="x")
        message_label.bind("<Configure>", lambda event: message_label.configure(wraplength=max(event.width - 48, 360)))
        result = {"continue": False}

        def continue_action() -> None:
            result["continue"] = True
            self.root.quit()

        ttk.Button(self._template_actions, text=continue_label, style="Primary.TButton", command=continue_action).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        self.root.mainloop()
        return result["continue"] and not self._exit_requested

    def show_check_grid(self, items: list[InstallItem]) -> None:
        if self._template_media is None:
            return
        for child in self._template_media.winfo_children():
            child.destroy()
        grid = tk.Frame(self._template_media, bg=self.SURFACE)
        grid.pack(fill="x", padx=4, pady=(20, 0))
        for column in range(self.GRID_COLUMNS):
            grid.grid_columnconfigure(column, weight=1, uniform="dependencies")
        for index, item in enumerate(items):
            row, column = divmod(index, self.GRID_COLUMNS)
            cell = tk.Label(grid, text=f"•  {item.name}", bg=self.SURFACE_2, fg=self.MUTED, font=("DejaVu Sans", 10, "bold"), justify="left", anchor="w", padx=14, pady=12, highlightbackground=self.BORDER, highlightthickness=1)
            cell.grid(row=row, column=column, sticky="ew", padx=5, pady=5)
            self._check_cells[item.name] = cell
        self.root.update_idletasks()
        self.root.update()

    def mark_check_result(self, item: InstallItem, installed: bool) -> None:
        cell = self._check_cells.get(item.name)
        if cell is None:
            return
        if installed:
            cell.configure(text=f"✓  {item.name}", bg=self.GREEN_DARK, fg="#c8f7d5", highlightbackground=self.GREEN_BORDER)
        else:
            cell.configure(text=f"!  {item.name}", bg="#2a171b", fg=self.RED_PALE, highlightbackground=self.RED_DARK)
        self.root.update_idletasks()
        self.root.update()

    def pause_after_check_screen(self, items: list[InstallItem]) -> bool:
        if self._template_actions is None:
            raise InstallerError("The prerequisite-check template has not been initialized.")
        self.status_var.set("Prerequisite check complete")
        if self._template_progress is not None:
            self._template_progress.configure(value=100)
        if self._template_status is not None:
            self._template_status.configure(text="Check complete", fg=self.GREEN)
        for child in self._template_actions.winfo_children():
            child.destroy()
        missing = sum(not bool(item.installed) for item in items)
        summary = "All prerequisites found" if missing == 0 else f"{missing} prerequisite{'s' if missing != 1 else ''} require installation"
        tk.Label(self._template_actions, text=summary, bg=self.SURFACE, fg=self.MUTED, font=("DejaVu Sans", 10), anchor="w").pack(side="left")
        result = {"continue": False}

        def continue_action() -> None:
            result["continue"] = True
            self.root.quit()

        ttk.Button(self._template_actions, text="Continue", style="Primary.TButton", command=continue_action).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        self.root.mainloop()
        return result["continue"] and not self._exit_requested

    def welcome_and_license(self, license_content: str) -> bool:
        self._clear_card()
        self.status_var.set("Review the license agreement")
        result = {"accepted": False}
        self._heading("Administrator installation", "Prepare your ACSL simulation environment", "This simulator installer is running with administrator access. Review and accept the license agreement before any installation begins.")
        banner = tk.Frame(self.card, bg="#2a171b", highlightbackground=self.RED_DARK, highlightthickness=1)
        banner.pack(fill="x", pady=(0, 16))
        banner_text = tk.Label(banner, text=("ADMINISTRATOR ACCESS ACTIVE  •  System changes are permitted. " "No packages, builds, files, or configuration changes will be made " "until you accept this agreement and continue through later screens."), bg="#2a171b", fg=self.RED_PALE, font=("DejaVu Sans", 10, "bold"), justify="left", anchor="w", padx=14, pady=12)
        banner_text.pack(fill="x")
        banner_text.bind("<Configure>", lambda event: banner_text.configure(wraplength=max(event.width - 28, 360)))
        ttk.Label(self.card, text="License agreement", style="Title.TLabel").pack(anchor="w", pady=(0, 10))
        license_box = self._scroll_text(license_content)
        license_box.pack(fill="both", expand=True)
        agreed = tk.BooleanVar(value=False)
        accept_button: Optional[ttk.Button] = None

        def update_accept_button() -> None:
            if accept_button is not None:
                accept_button.configure(state="normal" if agreed.get() else "disabled")

        ttk.Checkbutton(self.card, text="I have read and accept the license agreement.", variable=agreed, command=update_accept_button, style="Choice.TCheckbutton").pack(anchor="w", pady=(16, 0))
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", pady=(18, 0))

        def decline() -> None:
            result["accepted"] = False
            self.root.quit()

        def accept() -> None:
            result["accepted"] = True
            self.root.quit()

        ttk.Button(actions, text="Decline and exit", style="Secondary.TButton", command=decline).pack(side="right", padx=(10, 0))
        accept_button = ttk.Button(actions, text="Agree and continue", style="Primary.TButton", command=accept, state="disabled")
        accept_button.pack(side="right")
        update_accept_button()
        self.root.protocol("WM_DELETE_WINDOW", decline)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        return result["accepted"] and not self._exit_requested

    def update_prompt(self) -> bool:
        self._clear_card()
        self.status_var.set("Ubuntu package update recommendation")
        result = {"approved": False}
        self._heading("Recommended before installation", "Update Ubuntu packages", "Refreshing package metadata and applying ordinary available Ubuntu updates helps prerequisite packages install cleanly.")
        notice = tk.Frame(self.card, bg="#181818", highlightbackground=self.BORDER, highlightthickness=1)
        notice.pack(fill="x", pady=(0, 18))
        notice_text = tk.Label(notice, text=("If approved, the installer will run:\n\napt update\napt upgrade -y\n\n" "This refreshes package metadata and installs ordinary available updates. " "The installer does not run apt full-upgrade."), bg="#181818", fg=self.TEXT, font=("DejaVu Sans", 10), justify="left", anchor="w", padx=16, pady=14)
        notice_text.pack(fill="x")
        notice_text.bind("<Configure>", lambda event: notice_text.configure(wraplength=max(event.width - 32, 360)))
        tk.Label(self.card, text="Recommendation: update packages now unless you intentionally manage Ubuntu updates separately.", bg=self.SURFACE, fg=self.AMBER, font=("DejaVu Sans", 10, "bold"), justify="left", anchor="w").pack(fill="x", pady=(2, 0))
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(26, 0))

        def skip() -> None:
            result["approved"] = False
            self.root.quit()

        def approve() -> None:
            result["approved"] = True
            self.root.quit()

        ttk.Button(actions, text="Skip for now", style="Secondary.TButton", command=skip).pack(side="right", padx=(10, 0))
        ttk.Button(actions, text="Update packages", style="Primary.TButton", command=approve).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", skip)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        return result["approved"] and not self._exit_requested

    def package_selection_screen(self, items: list[InstallItem]) -> Optional[list[InstallItem]]:
        self._clear_card()
        self.status_var.set("Select prerequisites")
        result: dict[str, Optional[list[InstallItem]]] = {"selected": None}
        self._heading("Installation plan", "Select prerequisite packages", "Missing components are selected. Click a tile to change the selection.")
        picker = tk.Frame(self.card, bg=self.SURFACE)
        picker.pack(anchor="n", pady=(2, 0))
        for column in range(self.GRID_COLUMNS):
            picker.grid_columnconfigure(column, minsize=self.PICKER_TILE_WIDTH, weight=0)

        selected_vars: dict[str, tk.BooleanVar] = {}
        tiles: dict[str, tk.Frame] = {}
        name_labels: dict[str, tk.Label] = {}
        detail_labels: dict[str, tk.Label] = {}
        state_labels: dict[str, tk.Label] = {}

        def paint_tile(item: InstallItem) -> None:
            selected = selected_vars[item.name].get()
            tile, name = tiles[item.name], name_labels[item.name]
            detail, state = detail_labels[item.name], state_labels[item.name]
            if selected:
                tile.configure(bg=self.GREEN_DARK, highlightbackground=self.GREEN_BORDER)
                name.configure(bg=self.GREEN_DARK, fg="#e4ffec")
                detail.configure(bg=self.GREEN_DARK, fg="#b7f7c9")
                state.configure(bg=self.GREEN_DARK, fg=self.GREEN, text="Selected")
            else:
                tile.configure(bg=self.SURFACE, highlightbackground=self.BORDER)
                name.configure(bg=self.SURFACE, fg=self.TEXT)
                detail.configure(bg=self.SURFACE, fg=self.MUTED)
                state.configure(bg=self.SURFACE, fg=self.DIM, text="Installed" if item.installed else "Available")

        def toggle_item(item: InstallItem) -> None:
            selected_vars[item.name].set(not selected_vars[item.name].get())
            paint_tile(item)

        for index, item in enumerate(items):
            row, column = divmod(index, self.GRID_COLUMNS)
            selected_vars[item.name] = tk.BooleanVar(value=item.selected)
            tile = tk.Frame(picker, bg=self.SURFACE, width=self.PICKER_TILE_WIDTH, height=self.PICKER_TILE_HEIGHT, highlightbackground=self.BORDER, highlightthickness=1, cursor="hand2")
            tile.grid(row=row, column=column, sticky="n", padx=5, pady=5)
            tile.grid_propagate(False)
            tiles[item.name] = tile
            name_text = item.name if len(item.name) <= self.PICKER_NAME_MAX_CHARS else item.name[: self.PICKER_NAME_MAX_CHARS - 1] + "…"
            name = tk.Label(tile, text=name_text, bg=self.SURFACE, fg=self.TEXT, font=("DejaVu Sans", 8, "bold"), justify="left", anchor="w", cursor="hand2")
            name.place(x=10, y=7, width=self.PICKER_TILE_WIDTH - 20, height=14)
            name_labels[item.name] = name
            description = item.description
            if len(description) > self.PICKER_DESCRIPTION_MAX_CHARS:
                description = description[: self.PICKER_DESCRIPTION_MAX_CHARS - 1].rstrip() + "…"
            detail = tk.Label(tile, text=description, bg=self.SURFACE, fg=self.MUTED, font=("DejaVu Sans", 7), justify="left", anchor="w", cursor="hand2")
            detail.place(x=10, y=23, width=self.PICKER_TILE_WIDTH - 20, height=12)
            detail_labels[item.name] = detail
            state = tk.Label(tile, text="Installed" if item.installed else "Available", bg=self.SURFACE, fg=self.DIM, font=("DejaVu Sans", 7, "bold"), justify="left", anchor="w", cursor="hand2")
            state.place(x=10, y=40, width=self.PICKER_TILE_WIDTH - 20, height=12)
            state_labels[item.name] = state
            for widget in (tile, name, detail, state):
                widget.bind("<Button-1>", lambda _event, current=item: toggle_item(current))
            paint_tile(item)

        bottom = ttk.Frame(self.card, style="Surface.TFrame")
        bottom.pack(fill="x", side="bottom", pady=(18, 0))
        tk.Label(bottom, text="Press Continue to review and install the pre-requisites.", bg=self.SURFACE, fg=self.MUTED, font=("DejaVu Sans", 10), anchor="w").pack(side="left")

        def exit_selection() -> None:
            result["selected"] = None
            self.root.quit()

        def continue_action() -> None:
            for item in items:
                item.selected = selected_vars[item.name].get()
            result["selected"] = [item for item in items if item.selected]
            self.root.quit()

        ttk.Button(bottom, text="Exit", style="Secondary.TButton", command=exit_selection).pack(side="right", padx=(10, 0))
        ttk.Button(bottom, text="Continue", style="Primary.TButton", command=continue_action).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", exit_selection)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        return result["selected"]

    def install_review_screen(self, selected: list[InstallItem]) -> bool:
        self._clear_card()
        self.status_var.set("Review installation plan")
        result = {"start": False}
        self._heading("Installation plan", "Install selected prerequisites", "Review the selected components before installation begins.")
        selected_text = "\n".join(f"• {item.name}" for item in selected) if selected else "No prerequisites selected."
        summary = self._scroll_text(selected_text, height=14)
        summary.pack(fill="both", expand=True)
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", pady=(18, 0))
        tk.Label(actions, text="Press Start installation to continue.", bg=self.SURFACE, fg=self.MUTED, font=("DejaVu Sans", 10), anchor="w").pack(side="left")

        def cancel() -> None:
            result["start"] = False
            self.root.quit()

        def start() -> None:
            result["start"] = True
            self.root.quit()

        ttk.Button(actions, text="Exit", style="Secondary.TButton", command=cancel).pack(side="right", padx=(10, 0))
        ttk.Button(actions, text="Start installation", style="Primary.TButton", command=start).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", cancel)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        return result["start"] and not self._exit_requested

    def chrono_final_build_prompt(self, build_directory: Path) -> bool:
        """Ask before starting the final Project Chrono compilation."""
        self._clear_card()
        self.status_var.set("Project Chrono ready to compile")
        result = {"build": False}

        self._heading(
            "Project Chrono",
            "Final configuration is complete",
            (
                "Project Chrono 9.0.0 is configured and ready to compile. "
                "Select Go ahead and compile to build it now."
            ),
        )

        panel = tk.Frame(
            self.card,
            bg=self.GREEN_DARK,
            highlightbackground=self.GREEN_BORDER,
            highlightthickness=1,
        )
        panel.pack(fill="x", pady=(22, 0))

        tk.Label(
            panel,
            text=(
                "PROJECT CHRONO 9.0.0 READY\n\n"
                f"Build directory:\n{build_directory}\n\n"
                f"The installer will run:\n"
                f"cmake --build . --parallel {build_jobs()}"
            ),
            bg=self.GREEN_DARK,
            fg="#c8f7d5",
            font=("DejaVu Sans", 11, "bold"),
            justify="left",
            anchor="w",
            padx=18,
            pady=18,
        ).pack(fill="x")

        tk.Label(
            self.card,
            text=(
                "This compiles the configured Project Chrono build tree. "
                "It does not install files system-wide."
            ),
            bg=self.SURFACE,
            fg=self.AMBER,
            font=("DejaVu Sans", 10, "bold"),
            justify="left",
            anchor="w",
        ).pack(fill="x", pady=(18, 0))

        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(30, 0))

        def skip() -> None:
            result["build"] = False
            self.root.quit()

        def build() -> None:
            result["build"] = True
            self.root.quit()

        ttk.Button(
            actions,
            text="Not now",
            style="Secondary.TButton",
            command=skip,
        ).pack(side="right", padx=(10, 0))

        ttk.Button(
            actions,
            text="Go ahead and compile",
            style="Primary.TButton",
            command=build,
        ).pack(side="right")

        self.root.protocol("WM_DELETE_WINDOW", skip)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)

        return result["build"] and not self._exit_requested

    def ros_workspace_rebuild_prompt(self, workspace: Path) -> bool:
        """Return True when the user requests a clean ROS workspace rebuild."""
        self._clear_card()
        self.status_var.set("ROS 2 workspace detected")
        result = {"rebuild": False}

        self._heading(
            "ROS 2 workspace",
            "chrono-ros-messages build detected",
            (
                "Previous generated ROS 2 workspace output was found. You may keep "
                "the current workspace build or remove the generated build, install, "
                "and log directories and rebuild from source."
            ),
        )

        panel = tk.Frame(
            self.card,
            bg=self.SURFACE_2,
            highlightbackground=self.BORDER,
            highlightthickness=1,
        )
        panel.pack(fill="x", pady=(18, 0))

        tk.Label(
            panel,
            text=(
                f"Workspace:\n{workspace}\n\n"
                "Detected generated directories:\n"
                "• build/\n"
                "• install/\n"
                "• log/"
            ),
            bg=self.SURFACE_2,
            fg=self.TEXT,
            font=("DejaVu Sans Mono", 10),
            justify="left",
            anchor="w",
            padx=16,
            pady=16,
        ).pack(fill="x")

        tk.Label(
            self.card,
            text=(
                "Clean and rebuild removes only build/, install/, and log/ "
                "inside this workspace. Source files in src/ are preserved."
            ),
            bg=self.SURFACE,
            fg=self.AMBER,
            font=("DejaVu Sans", 10, "bold"),
            justify="left",
            anchor="w",
        ).pack(fill="x", pady=(18, 0))

        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(30, 0))

        def skip() -> None:
            result["rebuild"] = False
            self.root.quit()

        def rebuild() -> None:
            result["rebuild"] = True
            self.root.quit()

        ttk.Button(
            actions,
            text="Skip rebuild",
            style="Secondary.TButton",
            command=skip,
        ).pack(side="right", padx=(10, 0))

        ttk.Button(
            actions,
            text="Clean and rebuild",
            style="Primary.TButton",
            command=rebuild,
        ).pack(side="right")

        self.root.protocol("WM_DELETE_WINDOW", skip)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)

        return result["rebuild"] and not self._exit_requested

    def ros_detected_screen(self, distribution: str) -> bool:
        self._clear_card()
        self.status_var.set("ROS 2 detected")
        result = {"continue": False}
        self._heading("ROS 2", f"ROS 2 {distribution.title()} detected", "The required ROS 2 base installation is already available.")
        panel = tk.Frame(self.card, bg=self.GREEN_DARK, highlightbackground=self.GREEN_BORDER, highlightthickness=1)
        panel.pack(fill="x", pady=(28, 0))
        tk.Label(panel, text=f"ROS 2 {distribution.upper()} READY\n\n/opt/ros/{distribution}/setup.bash", bg=self.GREEN_DARK, fg="#c8f7d5", font=("DejaVu Sans", 12, "bold"), justify="center", anchor="center", padx=24, pady=22).pack(fill="x")
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(30, 0))

        def continue_action() -> None:
            result["continue"] = True
            self.root.quit()

        ttk.Button(actions, text="Continue", style="Primary.TButton", command=continue_action).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        self.root.mainloop()
        return result["continue"] and not self._exit_requested

    def ros_install_prompt(self, distribution: str) -> bool:
        self._clear_card()
        self.status_var.set("ROS 2 installation")
        result = {"install": False}
        self._heading("ROS 2", f"Install ROS 2 {distribution.title()}", "ROS 2 was not detected on this system.")
        panel = tk.Frame(self.card, bg=self.SURFACE_2, highlightbackground=self.BORDER, highlightthickness=1)
        panel.pack(fill="x", pady=(20, 0))
        tk.Label(panel, text=f"The installer will add the ROS 2 package source and install:\n\nros-{distribution}-ros-base\npython3-argcomplete\nros-dev-tools", bg=self.SURFACE_2, fg=self.TEXT, font=("DejaVu Sans", 10), justify="left", anchor="w", padx=16, pady=16).pack(fill="x")
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(30, 0))

        def skip() -> None:
            result["install"] = False
            self.root.quit()

        def install() -> None:
            result["install"] = True
            self.root.quit()

        ttk.Button(actions, text="Skip ROS 2", style="Secondary.TButton", command=skip).pack(side="right", padx=(10, 0))
        ttk.Button(actions, text="Install ROS 2", style="Primary.TButton", command=install).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", skip)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        return result["install"] and not self._exit_requested

    def ros_bashrc_prompt(self, distribution: str, bashrc: Path) -> bool:
        self._clear_card()
        self.status_var.set("ROS 2 shell setup")
        result = {"add": False}
        source_line = f"source /opt/ros/{distribution}/setup.bash"
        self._heading("ROS 2", "Enable ROS 2 in new terminals", "Add the ROS 2 environment setup line to your Bash configuration.")
        panel = tk.Frame(self.card, bg=self.SURFACE_2, highlightbackground=self.BORDER, highlightthickness=1)
        panel.pack(fill="x", pady=(20, 0))
        tk.Label(panel, text=f"File:\n{bashrc}\n\nLine to add:\n{source_line}", bg=self.SURFACE_2, fg=self.TEXT, font=("DejaVu Sans Mono", 10), justify="left", anchor="w", padx=16, pady=16).pack(fill="x")
        tk.Label(self.card, text="Skip this if you manage multiple ROS 2 distributions manually.", bg=self.SURFACE, fg=self.AMBER, font=("DejaVu Sans", 10, "bold"), justify="left", anchor="w").pack(fill="x", pady=(18, 0))
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(30, 0))

        def skip() -> None:
            result["add"] = False
            self.root.quit()

        def add() -> None:
            result["add"] = True
            self.root.quit()

        ttk.Button(actions, text="Not now", style="Secondary.TButton", command=skip).pack(side="right", padx=(10, 0))
        ttk.Button(actions, text="Add to .bashrc", style="Primary.TButton", command=add).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", skip)
        self.root.mainloop()
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        return result["add"] and not self._exit_requested

    def ros_complete_screen(self, distribution: str, bashrc_added: bool) -> bool:
        self._clear_card()
        self.status_var.set("ROS 2 complete")
        result = {"continue": False}
        self._heading("ROS 2", f"ROS 2 {distribution.title()} is ready", "The ROS 2 base environment is available for the ACSL workflow.")
        details = f"ROS 2 {distribution.title()} is installed.\n\nEnvironment script:\nsource /opt/ros/{distribution}/setup.bash"
        details += "\n\nThe environment script was added to your .bashrc." if bashrc_added else "\n\nSource the environment script manually in each terminal."
        panel = tk.Frame(self.card, bg=self.GREEN_DARK, highlightbackground=self.GREEN_BORDER, highlightthickness=1)
        panel.pack(fill="x", pady=(24, 0))
        label = tk.Label(panel, text=details, bg=self.GREEN_DARK, fg="#c8f7d5", font=("DejaVu Sans", 11, "bold"), justify="left", anchor="w", padx=18, pady=18)
        label.pack(fill="x")
        label.bind("<Configure>", lambda event: label.configure(wraplength=max(event.width - 36, 360)))
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", side="bottom", pady=(30, 0))

        def continue_action() -> None:
            result["continue"] = True
            self.root.quit()

        ttk.Button(actions, text="Continue", style="Primary.TButton", command=continue_action).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self._exit)
        self.root.mainloop()
        return result["continue"] and not self._exit_requested

    def error_screen(self, component: str, completed: list[str], error: Exception) -> None:
        self._clear_card()
        self.status_var.set("Installation failed")
        self._heading("Installation failed", f"{component} could not be installed", "The installer stopped before running later components.")
        panel = tk.Frame(self.card, bg="#2a171b", highlightbackground=self.RED_DARK, highlightthickness=1)
        panel.pack(fill="x", pady=(8, 22))
        detail = "Review the terminal where this installer was launched for complete apt, CMake, compiler, or build output.\n\n" f"Reported error: {error}"
        label = tk.Label(panel, text=detail, bg="#2a171b", fg=self.RED_PALE, font=("DejaVu Sans", 10, "bold"), justify="left", anchor="w", padx=16, pady=14)
        label.pack(fill="x")
        label.bind("<Configure>", lambda event: label.configure(wraplength=max(event.width - 32, 360)))
        ttk.Label(self.card, text="Completed components", style="Title.TLabel").pack(anchor="w", pady=(0, 10))
        completed_text = "\n".join(f"✓  {name}" for name in completed) if completed else "No components completed before the failure."
        completed_box = self._scroll_text(completed_text, height=11)
        completed_box.pack(fill="both", expand=True)
        actions = ttk.Frame(self.card, style="Surface.TFrame")
        actions.pack(fill="x", pady=(18, 0))
        ttk.Button(actions, text="Close installer", style="Primary.TButton", command=self.root.quit).pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)
        self.root.mainloop()

    def close(self) -> None:
        self._stop_gif()
        try:
            if self.root.winfo_exists():
                self.root.destroy()
        except tk.TclError:
            pass


def install_thrust() -> None:
    apt_install(["libthrust-dev"])


def install_python3() -> None:
    apt_install(["python3"])


def install_pip3() -> None:
    apt_install(["python3-pip"])


def install_gcc() -> None:
    apt_install(["build-essential"])


def install_clang() -> None:
    apt_install(["clang"])


def install_cmake() -> None:
    apt_install(["openssl", "libssl-dev", "pkg-config"])
    source = REPO_DIR / "libraries" / "third-party" / "CMake"
    if not source.is_dir():
        raise InstallerError(f"CMake source directory was not found:\n\n{source}")
    run(["./bootstrap"], cwd=source)
    run(["make", f"-j{build_jobs()}"], cwd=source)
    run(["make", "install"], cwd=source)
    apt_install(["cmake-curses-gui"])
    run(["ldconfig"])


def install_eigen3() -> None:
    apt_install(["libeigen3-dev"])


def install_irrlicht() -> None:
    apt_install(["libirrlicht1.8", "libirrlicht-dev", "libirrlicht-doc"])


def install_blaze() -> None:
    source = REPO_DIR / "libraries" / "third-party" / "blaze" / "blaze"
    if not source.is_dir():
        raise InstallerError(f"Blaze source directory was not found:\n\n{source}")
    shutil.copytree(source, Path("/usr/local/include/blaze"), dirs_exist_ok=True)
    run(["ldconfig"])


def install_boost() -> None:
    source = REPO_DIR / "libraries" / "third-party" / "boost"
    if not source.is_dir():
        raise InstallerError(f"Boost source directory was not found:\n\n{source}")
    run(["./bootstrap.sh"], cwd=source)
    run(["./b2", f"-j{build_jobs()}"], cwd=source)
    run(["./b2", "install", f"-j{build_jobs()}"], cwd=source)
    run(["ldconfig"])


def install_glm() -> None:
    source = REPO_DIR / "libraries" / "third-party" / "glm-0.9.9.8" / "glm"
    if not source.is_dir():
        raise InstallerError(f"GLM source directory was not found:\n\n{source}")
    shutil.copytree(source, Path("/usr/local/include/glm"), dirs_exist_ok=True)
    run(["ldconfig"])


def build_and_install(source: Path, build: Path, cmake_command: list[str]) -> None:
    """Create an ignored build directory; preserve it on failed configuration/build."""
    if not source.is_dir():
        raise InstallerError(f"Source directory was not found:\n\n{source}")
    ensure_ignored_directory(build)
    run(cmake_command, cwd=build)
    run(["make", f"-j{build_jobs()}"], cwd=build)
    run(["make", "install"], cwd=build)
    shutil.rmtree(build, ignore_errors=True)
    run(["ldconfig"])


def install_glfw() -> None:
    source = REPO_DIR / "libraries" / "third-party" / "glfw-3.3.8"
    build_and_install(source, source / "temp", ["cmake", ".."])


def install_glew() -> None:
    source = REPO_DIR / "libraries" / "third-party" / "glew-2.1.0"
    build_and_install(source, source / "temp", ["cmake", "../build/cmake"])
    glew_pc = source / "glew.pc"
    if glew_pc.is_file():
        glew_pc.unlink()


def install_opengl() -> None:
    apt_install(["libglu1-mesa-dev", "freeglut3-dev", "mesa-common-dev"])
    run(["ldconfig"])


def install_opencascade() -> None:
    apt_install(["libtool", "autoconf", "automake", "gfortran", "gdebi", "gcc-multilib", "libxi-dev", "libxmu-dev", "libxmu-headers", "libx11-dev", "mesa-common-dev", "libglu1-mesa-dev", "libfontconfig1-dev", "libfreetype6", "libfreetype6-dev", "tcl", "tcl-dev", "tk", "tk-dev"])
    source = REPO_DIR / "libraries" / "third-party" / "opencascade-7.4.0"
    build_and_install(source, source / "build", ["cmake", ".."])


def install_librealsense() -> None:
    apt_install(["libusb-1.0-0-dev", "libglfw3-dev", "libgl1-mesa-dev", "libglu1-mesa-dev", "libssl-dev"])
    source = REPO_DIR / "libraries" / "third-party" / "librealsense-2.53.1"
    build_and_install(source, source / "build", ["cmake", "..", "-DFORCE_RSUSB_BACKEND=true", "-DCMAKE_BUILD_TYPE=release"])
    rules = source / "config" / "99-realsense-libusb.rules"
    if rules.is_file():
        shutil.copy2(rules, Path("/etc/udev/rules.d/99-realsense-libusb.rules"))
        run(["udevadm", "control", "--reload-rules"])
        run(["udevadm", "trigger"])
    run(["ldconfig"])



VSG_CACHE_DIR = SCRIPT_DIR / "cache" / "vsg"
VSG_INSTALL_PREFIX = Path("/usr/local")
VSG_REPOSITORIES = {
    "assimp": ("https://github.com/assimp/assimp.git", "v5.3.1"),
    "vsg": ("https://github.com/vsg-dev/VulkanSceneGraph.git", "v1.1.4"),
    "vsgXchange": ("https://github.com/vsg-dev/vsgXchange.git", "v1.1.2"),
    "vsgImGui": ("https://github.com/vsg-dev/vsgImGui.git", "v0.6.0"),
    "vsgExamples": ("https://github.com/vsg-dev/vsgExamples.git", "v1.1.4"),
}
VSG_CMAKE_CONFIGS = {
    "vsg": "vsgConfig.cmake",
    "vsgXchange": "vsgXchangeConfig.cmake",
    "vsgImGui": "vsgImGuiConfig.cmake",
}


def ensure_vsg_cache_directory() -> Path:
    cache_root = SCRIPT_DIR / "cache"
    cache_root.mkdir(parents=True, exist_ok=True)
    root_ignore = cache_root / ".gitignore"
    if not root_ignore.exists():
        root_ignore.write_text("# Installer cache.\n*\n!.gitignore\n", encoding="utf-8")

    VSG_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    (VSG_CACHE_DIR / ".gitignore").write_text(
        "# Generated VSG source and build directories.\n*\n!.gitignore\n",
        encoding="utf-8",
    )
    return VSG_CACHE_DIR


def vsg_config_path(package: str) -> Optional[Path]:
    config_name = VSG_CMAKE_CONFIGS[package]
    candidates: list[Path] = []
    for library_root in (VSG_INSTALL_PREFIX / "lib", VSG_INSTALL_PREFIX / "lib64"):
        candidates.extend(path for path in library_root.glob(f"cmake/{package}*/{config_name}") if path.is_file())
    return sorted(set(candidates))[0] if candidates else None


def vsg_is_installed() -> bool:
    return all(vsg_config_path(package) is not None for package in VSG_CMAKE_CONFIGS)


def remove_old_vsg_installation() -> None:
    exact_names = {"vsg", "vsgXchange", "vsgImGui", "vsgExamples", "VulkanSceneGraph"}
    for root in (
        VSG_INSTALL_PREFIX / "bin",
        VSG_INSTALL_PREFIX / "include",
        VSG_INSTALL_PREFIX / "lib",
        VSG_INSTALL_PREFIX / "lib64",
        VSG_INSTALL_PREFIX / "share",
    ):
        if not root.is_dir():
            continue
        for child in list(root.iterdir()):
            lowered = child.name.lower()
            is_vsg_artifact = (
                child.name in exact_names
                or lowered.startswith("libvsg")
                or lowered.startswith("vsg")
                or "vulkanscenegraph" in lowered
            )
            if not is_vsg_artifact:
                continue
            if child.is_dir() and not child.is_symlink():
                shutil.rmtree(child)
            else:
                child.unlink(missing_ok=True)


def clone_vsg_source(name: str, destination: Path) -> None:
    repository, tag = VSG_REPOSITORIES[name]
    if destination.exists():
        shutil.rmtree(destination)
    run([
        "git", "clone", "-c", "advice.detachedHead=false", "--depth", "1",
        "--branch", tag, repository, str(destination),
    ])


def cmake_build_install(source: Path, build: Path, options: list[str]) -> None:
    """Configure, build, and install matching Release and Debug VSG artifacts."""
    if build.exists():
        shutil.rmtree(build)
    build.mkdir(parents=True, exist_ok=True)

    run([
        "cmake", "-S", str(source), "-B", str(build),
        "-G", "Ninja Multi-Config",
        "-DCMAKE_INSTALL_PREFIX=/usr/local",
        "-DCMAKE_DEBUG_POSTFIX=_d",
        "-DCMAKE_RELWITHDEBINFO_POSTFIX=_rd",
        *options,
    ])

    for configuration in ("Release", "Debug"):
        run([
            "cmake", "--build", str(build),
            "--config", configuration,
            "--parallel", str(build_jobs()),
        ])
        run([
            "cmake", "--install", str(build),
            "--config", configuration,
        ])


def install_vulkanscenegraph() -> None:
    apt_install([
        "git", "ninja-build", "libvulkan-dev", "vulkan-tools",
        "mesa-vulkan-drivers", "glslang-tools", "glslc", "libshaderc-dev",
        "libglfw3-dev", "libxinerama-dev", "libxcursor-dev", "libxi-dev",
        "libxrandr-dev", "libfreetype6-dev", "libfontconfig1-dev",
        "libdraco-dev", "zlib1g-dev",
    ])

    cache = ensure_vsg_cache_directory()
    sources = cache / "download_vsg"
    builds = cache / "build"
    sources.mkdir(parents=True, exist_ok=True)
    builds.mkdir(parents=True, exist_ok=True)

    # Deliberately remove only VSG-named content under /usr/local.
    # Assimp is left in place because unrelated software may use it.
    remove_old_vsg_installation()

    for name in VSG_REPOSITORIES:
        clone_vsg_source(name, sources / name)

    cmake_build_install(
        sources / "assimp",
        builds / "assimp",
        [
            "-DBUILD_SHARED_LIBS=OFF",
            "-DASSIMP_WARNINGS_AS_ERRORS=OFF",
            "-DASSIMP_BUILD_TESTS=OFF",
            "-DASSIMP_BUILD_ASSIMP_TOOLS=OFF",
            "-DASSIMP_BUILD_ZLIB=ON",
            "-DASSIMP_BUILD_DRACO=ON",
        ],
    )

    cmake_build_install(sources / "vsg", builds / "vsg", ["-DBUILD_SHARED_LIBS=ON"])
    vsg_config = vsg_config_path("vsg")
    if vsg_config is None:
        raise InstallerError("VSG installation failed: vsgConfig.cmake was not installed below /usr/local.")

    assimp_configs = list((VSG_INSTALL_PREFIX / "lib").glob("cmake/assimp*/assimpConfig.cmake"))
    assimp_configs += list((VSG_INSTALL_PREFIX / "lib64").glob("cmake/assimp*/assimpConfig.cmake"))
    if not assimp_configs:
        raise InstallerError("Assimp installation failed: assimpConfig.cmake was not installed below /usr/local.")

    cmake_build_install(
        sources / "vsgXchange",
        builds / "vsgXchange",
        [
            "-DBUILD_SHARED_LIBS=ON",
            f"-Dvsg_DIR={vsg_config.parent}",
            f"-Dassimp_DIR={sorted(assimp_configs)[0].parent}",
        ],
    )
    xchange_config = vsg_config_path("vsgXchange")
    if xchange_config is None:
        raise InstallerError("VSG installation failed: vsgXchangeConfig.cmake was not installed below /usr/local.")

    cmake_build_install(
        sources / "vsgImGui",
        builds / "vsgImGui",
        ["-DBUILD_SHARED_LIBS=ON", f"-Dvsg_DIR={vsg_config.parent}"],
    )
    imgui_config = vsg_config_path("vsgImGui")
    if imgui_config is None:
        raise InstallerError("VSG installation failed: vsgImGuiConfig.cmake was not installed below /usr/local.")

    cmake_build_install(
        sources / "vsgExamples",
        builds / "vsgExamples",
        [
            f"-Dvsg_DIR={vsg_config.parent}",
            f"-DvsgXchange_DIR={xchange_config.parent}",
            f"-DvsgImGui_DIR={imgui_config.parent}",
        ],
    )

    missing = [name for name in VSG_CMAKE_CONFIGS if vsg_config_path(name) is None]
    if missing:
        raise InstallerError(
            "VSG installation is incomplete. Missing CMake configurations below /usr/local:\n\n"
            + "\n".join(f"• {name}: {VSG_CMAKE_CONFIGS[name]}" for name in missing)
        )
    run(["ldconfig"])


def install_ros2_base(distribution: str) -> None:
    """Mirror the Bash ROS setup but install the requested base ROS package."""
    if distribution not in {"humble", "jazzy"}:
        raise InstallerError(f"Unsupported ROS 2 distribution: {distribution}")

    run(["apt", "update"])
    apt_install(["curl", "gnupg2", "lsb-release"])

    keyring = Path("/usr/share/keyrings/ros-archive-keyring.gpg")
    run(["curl", "-sSL", "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key", "-o", str(keyring)])

    architecture = subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip()
    codename = subprocess.check_output(["lsb_release", "-cs"], text=True).strip()
    source_line = (
        f"deb [arch={architecture} signed-by={keyring}] "
        f"http://packages.ros.org/ros2/ubuntu {codename} main\n"
    )
    Path("/etc/apt/sources.list.d/ros2.list").write_text(source_line, encoding="utf-8")

    run(["apt", "update"])
    run(["apt", "upgrade", "-y"])
    apt_install([f"ros-{distribution}-ros-base", "python3-argcomplete", "ros-dev-tools"])


def add_ros_to_bashrc(distribution: str) -> tuple[Path, bool]:
    """Add the selected ROS environment source line to the invoking user's .bashrc."""
    home = invoking_user_home()
    bashrc = home / ".bashrc"
    source_line = f"source /opt/ros/{distribution}/setup.bash"
    existing = bashrc.read_text(encoding="utf-8", errors="replace") if bashrc.exists() else ""

    if source_line in existing.splitlines():
        return bashrc, False

    with bashrc.open("a", encoding="utf-8") as stream:
        if existing and not existing.endswith("\n"):
            stream.write("\n")
        stream.write(f"\n# ACSL UAV Simulator: ROS 2 {distribution.title()}\n{source_line}\n")

    username = os.environ.get("SUDO_USER")
    if username:
        user = pwd.getpwnam(username)
        os.chown(bashrc, user.pw_uid, user.pw_gid)

    return bashrc, True


def make_install_items() -> list[InstallItem]:
    return [
        InstallItem("Thrust", "Parallel CUDA headers", lambda: dpkg_package_installed("thrust") or dpkg_package_installed("libthrust-dev"), install_thrust, "Ubuntu package"),
        InstallItem("Python 3", "Python runtime", lambda: command_exists("python3"), install_python3, "Ubuntu package"),
        InstallItem("pip for Python 3", "Python packages", lambda: command_exists("pip3"), install_pip3, "Ubuntu package"),
        InstallItem("GCC and build tools", "C and C++ compiler", lambda: command_exists("gcc") and command_exists("g++"), install_gcc, "Ubuntu package"),
        InstallItem("Clang", "LLVM compiler", lambda: command_exists("clang"), install_clang, "Ubuntu package"),
        InstallItem("CMake", "Build tools", cmake_is_sufficient, install_cmake, "Bundled source"),
        InstallItem("Eigen3", "Linear algebra", lambda: pkg_config_exists("eigen3"), install_eigen3, "Ubuntu package"),
        InstallItem("Irrlicht", "3D engine", lambda: dpkg_package_installed("libirrlicht-dev") or pkg_config_exists("irrlicht"), install_irrlicht, "Ubuntu package"),
        InstallItem("Blaze", "C++ math headers", lambda: Path("/usr/local/include/blaze").is_dir(), install_blaze, "Bundled source"),
        InstallItem("Boost", "C++ libraries", lambda: pkg_config_exists("boost") or Path("/usr/include/boost/version.hpp").is_file() or Path("/usr/local/include/boost/version.hpp").is_file(), install_boost, "Bundled source"),
        InstallItem("GLM", "OpenGL math", lambda: Path("/usr/local/include/glm").is_dir(), install_glm, "Bundled source"),
        InstallItem("GLFW", "Window and input", lambda: Path("/usr/local/include/GLFW").is_dir() and any_glob_exists("/usr/local/lib/libglfw*"), install_glfw, "Bundled source"),
        InstallItem("GLEW", "OpenGL loader", lambda: Path("/usr/local/include/GL/glew.h").is_file() and any_glob_exists("/usr/local/lib/libGLEW*"), install_glew, "Bundled source"),
        InstallItem("OpenGL", "Mesa development", lambda: pkg_config_exists("gl"), install_opengl, "Ubuntu package"),
        InstallItem("OpenCASCADE", "CAD toolkit", lambda: pkg_config_exists("occt") or Path("/usr/lib/libTKernel.so").is_file() or Path("/usr/local/lib/libTKernel.so").is_file(), install_opencascade, "Bundled source"),
        InstallItem("Librealsense", "RealSense SDK", lambda: Path("/usr/local/include/librealsense2").is_dir() or Path("/usr/local/lib/librealsense2.so").is_file(), install_librealsense, "Bundled source"),
        InstallItem("VulkanSceneGraph", "Vulkan renderer and Chrono VSG libraries", vsg_is_installed, install_vulkanscenegraph, "Source build"),
    ]


def update_system(ui: InstallerUI) -> None:
    ui.begin_work_screen("System update", "Updating Ubuntu packages", "Refreshing package metadata and applying available package updates.", "Updating")
    ui.show_template_animation()
    ui.run_background_task(lambda: (run(["apt", "update"]), run(["apt", "upgrade", "-y"])))


def check_prerequisites(ui: InstallerUI, items: list[InstallItem]) -> bool:
    total = len(items)
    ui.begin_work_screen("Environment check", "Checking prerequisites", "Scanning the ACSL and Project Chrono development environment.", "Checking", determinate=True)
    ui.show_check_grid(items)
    for index, item in enumerate(items, start=1):
        ui.set_work_status(f"Checking {item.name}", percent=((index - 1) * 100) // total)
        installed = item.refresh()
        ui.mark_check_result(item, installed)
        ui.set_work_status(f"Found {item.name}" if installed else f"Missing {item.name}", percent=(index * 100) // total)
    ui.set_work_status("Check complete", percent=100)
    return ui.pause_after_check_screen(items)


def install_selected_prerequisites(ui: InstallerUI, selected: list[InstallItem]) -> bool:
    installable = [item for item in selected if item.install is not None]
    deferred = [item for item in selected if item.install is None]
    if not installable:
        ui.begin_work_screen("Prerequisites", "No prerequisite installation required", "No installable prerequisites were selected.", "Complete", determinate=True)
        message = "PREREQUISITES COMPLETE\n\nNo prerequisite installation was required."
        if deferred:
            message += "\n\nVulkanSceneGraph remains deferred."
        return ui.finish_work_screen(message)

    ui.begin_work_screen("Prerequisites", "Installing prerequisites", "Installing selected ACSL and Project Chrono dependencies.", "Preparing installation", determinate=True)
    ui.show_template_animation()
    completed: list[str] = []
    total = len(installable)
    for index, item in enumerate(installable, start=1):
        ui.set_work_status(f"Installing {item.name}", percent=((index - 1) * 100) // total)
        try:
            assert item.install is not None
            ui.run_background_task(item.install)
        except Exception as error:
            raise ComponentInstallError(item.name, completed, error) from error
        item.installed = True
        completed.append(item.name)
        ui.set_work_status(f"Installed {item.name}", percent=(index * 100) // total)

    message = "PREREQUISITES COMPLETE\n\nSelected prerequisites have been installed successfully."
    if deferred:
        message += "\n\nVulkanSceneGraph remains deferred."
    return ui.finish_work_screen(message)



def ros_workspace_directory() -> Path:
    """Return the repository-local chrono ROS 2 workspace."""
    workspace = REPO_DIR / "libraries" / "chrono-ros-messages"
    if not workspace.is_dir():
        raise InstallerError(
            "The chrono-ros-messages workspace was not found:\n\n"
            f"{workspace}"
        )
    if not (workspace / "src").is_dir():
        raise InstallerError(
            "The chrono-ros-messages workspace has no src directory:\n\n"
            f"{workspace / 'src'}"
        )
    return workspace


def clean_ros_workspace(workspace: Path) -> None:
    """Remove only generated colcon output inside the known ROS workspace."""
    for directory_name in ("build", "install", "log"):
        generated = workspace / directory_name
        if generated.exists():
            shutil.rmtree(generated)


def build_chrono_ros_messages(distribution: str) -> None:
    """Clean and rebuild the chrono-ros-messages ROS 2 workspace."""
    if not ros_is_installed(distribution):
        raise InstallerError(
            f"ROS 2 {distribution.title()} is unavailable:\n\n"
            f"/opt/ros/{distribution}/setup.bash"
        )
    if not command_exists("colcon"):
        raise InstallerError(
            "The colcon command was not found.\n\n"
            "Install ros-dev-tools, then run the installer again."
        )

    workspace = ros_workspace_directory()
    clean_ros_workspace(workspace)

    shell_command = (
        "set -e; "
        f"source /opt/ros/{distribution}/setup.bash; "
        f"cd {str(workspace)!r}; "
        f"colcon build --parallel-workers {build_jobs()}"
    )
    run(["bash", "-lc", shell_command], cwd=workspace)


def ros_workspace_has_generated_output(workspace: Path) -> bool:
    """Return True only when a prior complete colcon output tree exists."""
    return all(
        (workspace / directory_name).is_dir()
        for directory_name in ("build", "install", "log")
    )


def compile_chrono_ros_messages(ui: InstallerUI, distribution: str) -> bool:
    """Offer to retain an existing workspace build or clean and rebuild it."""
    workspace = ros_workspace_directory()

    if ros_workspace_has_generated_output(workspace):
        if not ui.ros_workspace_rebuild_prompt(workspace):
            return True

    ui.begin_work_screen(
        "ROS 2 workspace",
        "Rebuilding chrono-ros-messages",
        (
            "The installer will remove only this workspace's generated build, "
            "install, and log directories before rebuilding the ROS 2 packages.\n\n"
            f"Workspace:\n{workspace}"
        ),
        "Preparing clean ROS 2 workspace build",
        determinate=True,
    )
    ui.show_template_animation()

    try:
        ui.run_background_task(lambda: build_chrono_ros_messages(distribution))
    except Exception as error:
        raise ComponentInstallError("chrono-ros-messages workspace", [], error) from error

    return ui.finish_work_screen(
        "ROS 2 WORKSPACE COMPLETE\n\n"
        "chrono-ros-messages was cleaned and rebuilt successfully."
    )



def chrono_source_directory() -> Path:
    """Return the Project Chrono source tree and verify it is complete."""
    source = REPO_DIR / "libraries" / "chrono"
    if not (source / "CMakeLists.txt").is_file():
        raise InstallerError(
            "The Project Chrono source directory was not found or is incomplete:\n\n"
            f"{source}"
        )
    return source


def chrono_build_directory() -> Path:
    """Create or reuse the persistent Chrono CMake build directory."""
    build = REPO_DIR / "libraries" / "chrono-build"
    build.mkdir(parents=True, exist_ok=True)

    gitkeep = build / ".gitkeep"
    if not gitkeep.exists():
        gitkeep.touch()

    gitignore = build / ".gitignore"
    if not gitignore.exists():
        gitignore.write_text(
            "# Keep this configured build directory in the repository.\n"
            "*\n"
            "!.gitignore\n"
            "!.gitkeep\n",
            encoding="utf-8",
        )

    return build


def configure_project_chrono(distribution: str) -> None:
    """Configure Project Chrono with ACSL simulator modules enabled."""
    if not ros_is_installed(distribution):
        raise InstallerError(
            f"ROS 2 {distribution.title()} is unavailable:\n\n"
            f"/opt/ros/{distribution}/setup.bash"
        )

    chrono_source = chrono_source_directory()
    chrono_build = chrono_build_directory()
    ros_overlay = (
        REPO_DIR
        / "libraries"
        / "chrono-ros-messages"
        / "install"
        / "local_setup.bash"
    )

    if not ros_overlay.is_file():
        raise InstallerError(
            "The chrono-ros-messages ROS 2 overlay was not found:\n\n"
            f"{ros_overlay}\n\n"
            "Rebuild chrono-ros-messages before configuring Project Chrono."
        )

    cmake_command = [
        "cmake",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DENABLE_MODULE_CASCADE=ON",
        "-DENABLE_MODULE_IRRLICHT=ON",
        "-DENABLE_MODULE_MULTICORE=ON",
        "-DENABLE_MODULE_OPENGL=ON",
        "-DENABLE_MODULE_POSTPROCESS=ON",
        "-DENABLE_MODULE_VEHICLE=ON",
        "-DENABLE_MODULE_VSG=ON",
        "../chrono",
    ]

    shell_command = (
        "set -e; "
        f"source /opt/ros/{distribution}/setup.bash; "
        f"source {str(ros_overlay)!r}; "
        f"cd {str(chrono_build)!r}; "
        + " ".join(cmake_command)
    )

    run(["bash", "-lc", shell_command], cwd=chrono_build)


def configure_project_chrono_ui(ui: InstallerUI, distribution: str) -> bool:
    """Run Chrono CMake configuration using the existing animated work screen."""
    chrono_source = chrono_source_directory()
    chrono_build = chrono_build_directory()

    ui.begin_work_screen(
        "Project Chrono",
        "Configuring final simulator build",
        (
            "The installer will configure Project Chrono with CASCADE, IRRLICHT, "
            "MULTICORE, OPENGL, POSTPROCESS, VEHICLE, and VSG enabled.\n\n"
            f"Source: {chrono_source}\n"
            f"Build directory: {chrono_build}"
        ),
        "Configuring Project Chrono",
        determinate=True,
    )
    ui.show_template_animation()

    try:
        ui.run_background_task(
            lambda: configure_project_chrono(distribution)
        )
    except Exception as error:
        raise ComponentInstallError(
            "Project Chrono configuration",
            [],
            error,
        ) from error

    return ui.finish_work_screen(
        "CHRONO CONFIGURATION COMPLETE\n\n"
        "Project Chrono is configured with CASCADE, IRRLICHT, MULTICORE, "
        "OPENGL, POSTPROCESS, VEHICLE, and VSG enabled.\n\n"
        "The simulator is ready for the final installation/build step."
    )



def build_project_chrono() -> None:
    """Build the configured Project Chrono tree with the installer job limit."""
    build_directory = chrono_build_directory()
    cache = build_directory / "CMakeCache.txt"

    if not cache.is_file():
        raise InstallerError(
            "Project Chrono has not been configured yet:\n\n"
            f"{cache}\n\n"
            "Configure Project Chrono before starting the final build."
        )

    run([
        "cmake",
        "--build",
        str(build_directory),
        "--parallel",
        str(build_jobs()),
    ])


def build_project_chrono_ui(ui: InstallerUI) -> bool:
    """Prompt for and execute the final Project Chrono compilation."""
    build_directory = chrono_build_directory()

    if not ui.chrono_final_build_prompt(build_directory):
        return True

    ui.begin_work_screen(
        "Project Chrono",
        "Compiling Project Chrono 9.0.0",
        (
            "Compiling the configured Release build with the same parallel "
            "job limit used throughout the installer.\n\n"
            f"Build directory: {build_directory}\n"
            f"Parallel jobs: {build_jobs()}"
        ),
        "Compiling Project Chrono",
        determinate=True,
    )
    ui.show_template_animation()

    try:
        ui.run_background_task(build_project_chrono)
    except Exception as error:
        raise ComponentInstallError(
            "Project Chrono compilation",
            [],
            error,
        ) from error

    return ui.finish_work_screen(
        "PROJECT CHRONO BUILD COMPLETE\n\n"
        "Project Chrono 9.0.0 compiled successfully.\n\n"
        "The simulator is ready for the final installation step."
    )


def configure_ros2(ui: InstallerUI) -> bool:
    """Mirror the Bash script's ROS stage, using the requested base packages."""
    distribution = ros_distribution()
    if distribution is None:
        return True

    if ros_is_installed(distribution):
        if not ui.ros_detected_screen(distribution):
            return False
    else:
        if not ui.ros_install_prompt(distribution):
            return True
        ui.begin_work_screen("ROS 2", f"Installing ROS 2 {distribution.title()}", "Installing ROS 2 base and development tools.", "Installing ROS 2")
        ui.show_template_animation()
        try:
            ui.run_background_task(lambda: install_ros2_base(distribution))
        except Exception as error:
            raise ComponentInstallError(f"ROS 2 {distribution.title()}", [], error) from error
        if not ui.finish_work_screen(f"ROS 2 {distribution.upper()} INSTALLED\n\nROS 2 base and development tools are ready."):
            return False

    bashrc = invoking_user_home() / ".bashrc"
    bashrc_added = False
    if ui.ros_bashrc_prompt(distribution, bashrc):
        try:
            _, bashrc_added = add_ros_to_bashrc(distribution)
        except Exception as error:
            raise ComponentInstallError("ROS 2 shell setup", [], error) from error

    return ui.ros_complete_screen(distribution, bashrc_added)


def execute_installation(ui: InstallerUI) -> None:
    if not ui.welcome_and_license(license_text()):
        return
    if ui.update_prompt():
        update_system(ui)
        if not ui.finish_work_screen("UPDATE COMPLETE\n\nUbuntu packages have been updated successfully."):
            return
    else:
        ui.begin_work_screen("System update", "Continuing without package updates", "No Ubuntu package update was performed.", "Update skipped", determinate=True)
        if not ui.finish_work_screen("UPDATE SKIPPED\n\nContinuing to prerequisite detection."):
            return

    items = make_install_items()
    if not check_prerequisites(ui, items):
        return
    selected = ui.package_selection_screen(items)
    if selected is None:
        return
    if not ui.install_review_screen(selected):
        return
    if not install_selected_prerequisites(ui, selected):
        return

    if not configure_ros2(ui):
        return

    distribution = ros_distribution()
    if distribution is None or not ros_is_installed(distribution):
        return
    if not compile_chrono_ros_messages(ui, distribution):
        return
    if not configure_project_chrono_ui(ui, distribution):
        return
    build_project_chrono_ui(ui)


def main() -> int:
    ui: Optional[InstallerUI] = None
    try:
        print("Starting installer", flush=True)
        validate_environment()
        ensure_gui_dependencies()
        ui = InstallerUI()
        execute_installation(ui)
        return 0
    except ComponentInstallError as error:
        if ui is not None:
            ui.error_screen(error.component, error.completed, error.cause)
        else:
            print(f"\nINSTALLATION FAILED: {error.component}\n", file=sys.stderr)
        return 1
    except InstallerError as error:
        print(f"\nINSTALLER ERROR:\n{error}\n", file=sys.stderr)
        return 1
    except subprocess.CalledProcessError as error:
        command = " ".join(str(part) for part in error.cmd)
        print(f"\nCOMMAND FAILED:\nExit status: {error.returncode}\nCommand: {command}\n", file=sys.stderr)
        return error.returncode or 1
    except Exception as error:
        print(f"\nUNEXPECTED INSTALLER ERROR:\n{error}\n", file=sys.stderr)
        return 1
    finally:
        if ui is not None:
            ui.close()


if __name__ == "__main__":
    raise SystemExit(main())
