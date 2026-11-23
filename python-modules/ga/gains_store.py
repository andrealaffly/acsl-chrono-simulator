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
# File:        gains_store.py
# Authors:     Giri M. Kumar
# Date:        October 18, 2026
# For info:    Andrea L'Afflitto
#              a.lafflitto@vt.edu
#
# Description:
#     Owns the controller's active gains file for the length of a GA session.
#
#     The simulator always reads chrono-assets/parameters/<platform>/<CTRL>/
#     gains_<CTRL>.json -- there is no command-line option to point it at a
#     different file -- so the GA writes each candidate's gains over that
#     file. That is only safe because of the following:
#
#       begin()    copies the original, byte for byte, to
#                  <CTRL>/baseline_tune/gains_<CTRL>_<timestamp>.json and
#                  writes a lock file (sim-log/.ga_session.lock) pointing
#                  at that copy.
#       finish()   writes the original back and removes the lock. The
#                  session calls it on every exit path: done, stopped,
#                  error, or a bug.
#       recover_stale_lock()
#                  if the GA process was killed so hard that finish() never
#                  ran (kill -9, power loss), the lock is still there with a
#                  dead PID in it. The next session -- or the GUI, every
#                  time it loads the GA page -- finds it and restores the
#                  snapshot before doing anything else.
#
#     Every write is atomic (write a temp file, then rename it over the
#     target), so a simulator starting up can never read a half-written
#     JSON.
###############################################################################

from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Optional

from .context import SIM_LOG_DIR, SimContext

LOCK_PATH = SIM_LOG_DIR / ".ga_session.lock"


class StoreError(Exception):
    """Another GA session is running, or a gains file can't be touched."""


# =============================================================================
# Atomic writes
# =============================================================================
def write_atomic(path: Path, payload: bytes) -> None:
    """Replace `path` with `payload` in one step.

    The bytes go to a temp file in the same folder, which is then renamed
    over `path`. A rename within one filesystem is atomic, so anyone reading
    `path` sees either the old file or the new one, never a mix.
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    # mkstemp creates its file owner-only (0600). Keep the mode the target
    # already had, or, for a new file, the usual umask-based one.
    # (os.umask can only be read by setting it, hence the set-and-restore.)
    if path.exists():
        mode = path.stat().st_mode & 0o777
    else:
        umask = os.umask(0)
        os.umask(umask)
        mode = 0o666 & ~umask

    handle, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(handle, "wb") as temp_file:
            temp_file.write(payload)
        os.chmod(temp_name, mode)
        os.replace(temp_name, path)
    except BaseException:
        # Including KeyboardInterrupt: never leave a stray temp file behind.
        if os.path.exists(temp_name):
            os.unlink(temp_name)
        raise


# =============================================================================
# The session lock
# =============================================================================
# The lock file is a small JSON:
#   {"pid": ..., "gains_path": ..., "snapshot_path": ..., "started": ...}
def _pid_alive(pid: int) -> bool:
    """True if a process with this PID exists (signal 0 only checks)."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        # It exists, it just belongs to someone else.
        return True
    return True


def read_lock() -> Optional[dict]:
    try:
        return json.loads(LOCK_PATH.read_text())
    except (OSError, ValueError):
        return None


def active_session_pid() -> Optional[int]:
    """PID of a GA session that is still running, or None."""
    lock = read_lock()
    if lock and isinstance(lock.get("pid"), int) and _pid_alive(lock["pid"]):
        return lock["pid"]
    return None


def recover_stale_lock() -> Optional[str]:
    """Undo what a dead session left behind: restore its snapshot over the
    gains file and remove the lock.

    Returns a message describing what was done, or None if there was nothing
    to do. Raises StoreError if the lock belongs to a session that is still
    running (that one must be stopped, not recovered).
    """
    if not LOCK_PATH.exists():
        return None
    lock = read_lock()
    if lock is None:
        LOCK_PATH.unlink()
        return f"Removed unreadable GA lock file {LOCK_PATH}"
    if active_session_pid() is not None:
        raise StoreError(f"Another GA session is running (PID {lock['pid']})")

    gains_path = Path(lock.get("gains_path", ""))
    snapshot_path = Path(lock.get("snapshot_path", ""))
    LOCK_PATH.unlink()
    if not snapshot_path.is_file():
        return f"Removed stale GA lock; its snapshot {snapshot_path} is missing, so {gains_path} was NOT restored"
    write_atomic(gains_path, snapshot_path.read_bytes())
    return f"Restored {gains_path} from {snapshot_path} (left behind by an interrupted GA session)"


# =============================================================================
# GainsStore
# =============================================================================
class GainsStore:
    """The active gains file of one session: snapshot, candidates, restore."""

    def __init__(self, ctx: SimContext, timestamp: str) -> None:
        self.ctx = ctx
        self.timestamp = timestamp
        self.gains_path = ctx.gains_path
        self.snapshot_path = ctx.baseline_dir / f"gains_{ctx.controller_name}_{timestamp}.json"
        self.original_bytes = b""   # the gains file as it was when the session began
        self._active = False        # True between begin() and finish()

    def begin(self) -> None:
        """Snapshot the original gains and take the lock."""
        if not self.gains_path.is_file():
            raise StoreError(f"Gains file not found: {self.gains_path}")
        self.original_bytes = self.gains_path.read_bytes()
        write_atomic(self.snapshot_path, self.original_bytes)
        lock = {"pid": os.getpid(), "gains_path": str(self.gains_path), "snapshot_path": str(self.snapshot_path),
                "started": time.strftime("%Y-%m-%d %H:%M:%S")}
        write_atomic(LOCK_PATH, json.dumps(lock, indent=2).encode())
        self._active = True

    def write_candidate(self, payload: bytes) -> None:
        """Put a candidate's gains where the simulator will read them."""
        write_atomic(self.gains_path, payload)

    def finish(self) -> bool:
        """Restore the original gains and release the lock.

        Safe to call more than once; returns True only on the call that
        actually restored the gains.
        """
        if not self._active:
            return False
        write_atomic(self.gains_path, self.original_bytes)
        LOCK_PATH.unlink(missing_ok=True)
        self._active = False
        return True

    def save_tuned(self, payload: bytes) -> Path:
        """Save the session's best gains to <CTRL>/ga_tuned/ and return the path."""
        path = self.ctx.tuned_dir / f"gains_{self.ctx.controller_name}_{self.timestamp}.json"
        write_atomic(path, payload)
        return path


def apply_gains_file(ctx: SimContext, source: Path) -> None:
    """Copy a gains file (a tuned result, or an old baseline) over the
    controller's active one. Used by the GUI's Apply button.

    Refuses while a GA session is running (it would be overwritten by the
    next candidate, and then by the restore), and refuses anything that
    isn't valid JSON.
    """
    if active_session_pid() is not None:
        raise StoreError("A GA session is running; stop it before applying gains")
    payload = source.read_bytes()
    json.loads(payload)
    write_atomic(ctx.gains_path, payload)
