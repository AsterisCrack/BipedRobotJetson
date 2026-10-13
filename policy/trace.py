"""Per-step recording of real policy runs, for post-mortems and sim-to-real comparison.

One trace = one RUNNING episode, from ``start()`` until it stops, faults, disarms or is
e-stopped. Saved as ``data/traces/<UTC time>_<model>_<reason>.npz``:

    t            [T]       s since start (monotonic)
    frame        [T, 50]   the proprio observation frame the policy saw (policy joint order)
    action_raw   [T, 12]   policy output before the safety rate limiter
    action       [T, 12]   action actually used (clipped + rate limited)
    targets_rad  [T, 12]   EMA-filtered joint targets sent to the servos (policy order)
    rate_limited [T]       1 if the rate limiter changed the action this step
    command      [T, 3]    velocity command
    tilt_rad     [T]
    cycle_ms     [T]       time spent in the previous policy step
    infer_ms     [T]
    meta         json str  model, bundle info, joint names, limits, end reason

Load with ``np.load(path)`` and ``json.loads(str(d["meta"]))``.
"""

from __future__ import annotations

import json
import logging
import os
import re
import threading
import time
from datetime import datetime, timezone

import numpy as np

logger = logging.getLogger(__name__)

MAX_STEPS = 15000          # 5 min at 50 Hz, ~10 MB uncompressed
_FIELDS = ("t", "frame", "action_raw", "action", "targets_rad", "rate_limited",
           "command", "tilt_rad", "cycle_ms", "infer_ms")
_NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]+\.npz$")


class TraceRecorder:
    def __init__(self, root: str):
        self.root = root
        self._lock = threading.Lock()
        self._rows: dict[str, list] | None = None
        self._meta: dict = {}
        self._t0 = 0.0

    @property
    def active(self) -> bool:
        return self._rows is not None

    def begin(self, meta: dict) -> None:
        with self._lock:
            self._rows = {k: [] for k in _FIELDS}
            self._meta = dict(meta)
            self._meta["started_utc"] = datetime.now(timezone.utc).isoformat()
            self._t0 = time.monotonic()

    def record(self, **values) -> None:
        """Append one control step. Called from the bus thread: cheap, never raises."""
        rows = self._rows
        if rows is None or len(rows["t"]) >= MAX_STEPS:
            return
        try:
            rows["t"].append(time.monotonic() - self._t0)
            for k in _FIELDS[1:]:
                rows[k].append(np.asarray(values.get(k, np.nan), dtype=np.float32).copy())
        except Exception:
            logger.exception("trace record failed; dropping this trace")
            self._rows = None

    def end(self, reason: str, extra: dict | None = None) -> None:
        """Finish the current trace (if any) and save it on a background thread."""
        with self._lock:
            rows, meta = self._rows, self._meta
            self._rows = None
        if not rows or not rows["t"]:
            return
        meta = {**meta, **(extra or {}), "end_reason": reason, "steps": len(rows["t"]),
                "truncated": len(rows["t"]) >= MAX_STEPS}
        threading.Thread(target=self._save, args=(rows, meta), daemon=True, name="trace-save").start()

    def _save(self, rows: dict, meta: dict) -> None:
        try:
            os.makedirs(self.root, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            model = re.sub(r"[^A-Za-z0-9_.\-]", "_", str(meta.get("model", "model")))[:60]
            why = re.sub(r"[^A-Za-z0-9_\-]", "_", str(meta.get("end_reason", "end")))[:30]
            path = os.path.join(self.root, f"{stamp}_{model}_{why}.npz")
            arrays = {k: np.stack(v) if k != "t" else np.asarray(v, dtype=np.float64) for k, v in rows.items()}
            np.savez_compressed(path, meta=np.array(json.dumps(meta, default=str)), **arrays)
            logger.info("saved policy trace %s (%d steps)", path, meta["steps"])
        except Exception:
            logger.exception("saving policy trace failed")

    # -- browsing -----------------------------------------------------------

    def list(self) -> list[dict]:
        if not os.path.isdir(self.root):
            return []
        out = []
        for name in sorted(os.listdir(self.root), reverse=True):
            if not _NAME_RE.match(name):
                continue
            p = os.path.join(self.root, name)
            entry = {"name": name, "bytes": os.path.getsize(p),
                     "modified": datetime.fromtimestamp(os.path.getmtime(p), timezone.utc).isoformat()}
            try:
                with np.load(p) as d:
                    m = json.loads(str(d["meta"]))
                entry.update(steps=m.get("steps"), end_reason=m.get("end_reason"), model=m.get("model"),
                             seconds=round(m.get("steps", 0) / 50.0, 1))
            except Exception:
                entry["error"] = "unreadable"
            out.append(entry)
        return out

    def path(self, name: str) -> str | None:
        """Resolve a trace name to a file inside root, or None. Rejects any path tricks."""
        if not _NAME_RE.match(name):
            return None
        p = os.path.abspath(os.path.join(self.root, name))
        if os.path.dirname(p) != os.path.abspath(self.root) or not os.path.isfile(p):
            return None
        return p
