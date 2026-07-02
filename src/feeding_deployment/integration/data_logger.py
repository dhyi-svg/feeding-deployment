"""Per-day data logging for the meal-assistance deployment.

Collects everything the robot learns about during a meal so that a day's data
can be released: key images shown to / captured by the robot, every user input
from the web interface, and structured events (task commands, predicted vs.
ground-truth preferences, transfer poses, etc.).

Layout (one directory per deployment day, keyed by user + day number):

    log/<user>/day_<NN>/
        metadata.json          # day, user, scenario, start/end time, counts
        user_inputs.jsonl      # every message received from the web interface
        events.jsonl           # structured events logged by the executive/skills
        images_index.jsonl     # one record per saved image (path + metadata)
        images/
            <skill>/           # one folder per HLA (e.g. open_microwave, acquire_bite),
                               #   files <run>_<name>.png (0_rgb, 0_depth, ...; reruns -> 1_*)
            webapp_images/     # everything shown to the user on the iPad, in display order
            ...

Design notes:
- Raw capture only. Face blurring / PII scrubbing is a separate export step.
- Every public method is best-effort and swallows its own errors: data logging
  must never crash an in-progress meal.
- Thread-safe: the web-interface ROS callback, the head-perception thread, and
  the main executive loop all log concurrently.
"""

from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import cv2

    _CV2_IMPORTED = True
except ModuleNotFoundError:
    _CV2_IMPORTED = False

try:
    import numpy as np

    _NUMPY_IMPORTED = True
except ModuleNotFoundError:
    _NUMPY_IMPORTED = False


class DataLogger:
    """Thread-safe, best-effort per-day data logger.

    When ``enabled`` is False (e.g. no ``--day`` was passed) every method is a
    no-op, so callers can log unconditionally without guarding on the day.
    """

    def __init__(
        self,
        state_dir: Path,
        day: int | None = None,
    ) -> None:
        # Shared cross-day state directory (log/<user>/): always valid, even when
        # per-day release logging is disabled. This is the single source of truth
        # for "where this user's logs live"; callers read it as `.state_dir`
        # instead of taking a separate log_dir argument.
        self.state_dir = Path(state_dir)
        self.user = self.state_dir.name
        self.day = day
        self.enabled = day is not None
        self._lock = threading.Lock()
        self._image_seq = 0

        # Per-HLA image organization, two-level prefix `images/<hla>/<run>[_<retry>]_<name>.png`:
        #   <run>   -- the full skill execution; `begin_hla` bumps it (0, 1, 2, ...).
        #   <retry> -- a re-detection within that execution; omitted for the first
        #              capture, then 1, 2, ... So a full run is `1_rgb`, a rerun
        #              within run 0 is `0_1_rgb`. Distinct names in one capture share
        #              the prefix (0_rgb, 0_depth). `webapp_images/` is a separate
        #              funnel keyed by its own monotonic counter.
        self._current_hla: str | None = None
        self._run: dict[str, int] = {}
        self._retry: dict[str, int] = {}
        self._group_used: dict[str, set] = {}
        self._webapp_seq = 0

        if not self.enabled:
            self.day_dir = None
            print("[data_logger] No day provided; per-day release logging is DISABLED.")
            return

        self.day_dir = self.state_dir / f"day_{day:02d}"
        self.images_dir = self.day_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)

        self._user_inputs_path = self.day_dir / "user_inputs.jsonl"
        self._events_path = self.day_dir / "events.jsonl"
        self._images_index_path = self.day_dir / "images_index.jsonl"
        self._metadata_path = self.day_dir / "metadata.json"

        self._write_metadata(closed=False)
        print(f"[data_logger] Logging day {day:02d} for user '{self.user}' to {self.day_dir}")

    # -- internal helpers ---------------------------------------------------

    @staticmethod
    def _timestamp() -> dict[str, Any]:
        now = time.time()
        return {"epoch": now, "iso": datetime.fromtimestamp(now).isoformat()}

    def _append_jsonl(self, path: Path, record: dict[str, Any]) -> None:
        # Caller must hold the lock.
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, default=str) + "\n")

    def _write_metadata(self, closed: bool) -> None:
        meta = {
            "user": self.user,
            "day": self.day,
            "images_logged": self._image_seq,
            "closed": closed,
        }
        if not closed:
            meta["started"] = self._timestamp()
        else:
            meta["ended"] = self._timestamp()
        # Merge with any existing metadata (preserve the original start time).
        try:
            if self._metadata_path.exists():
                existing = json.loads(self._metadata_path.read_text(encoding="utf-8"))
                existing.update(meta)
                meta = existing
        except Exception:  # noqa: BLE001 - never let metadata IO break logging
            pass
        self._metadata_path.write_text(json.dumps(meta, indent=2, default=str), encoding="utf-8")

    def _reserve_slot(self, name: str) -> tuple:
        """Reserve the next capture slot for ``name`` -> ``(seq, folder, run, retry, stem)``.

        Encapsulates the per-HLA run/retry grouping (see ``log_image``) so that
        non-image artifacts -- e.g. a ``detection_inputs.json`` sidecar -- can share the
        exact same ``<run>[_<retry>]_<name>`` prefix as the images captured in the same
        detection, and thus be joined back to them by (folder, run, retry). Acquires the
        lock itself; callers must not already hold it.
        """
        with self._lock:
            seq = self._image_seq
            self._image_seq += 1
            if name == "webapp":
                folder = "webapp_images"
                run: int | None = None
                retry: int | None = None
                stem = f"{self._webapp_seq}_webapp"
                self._webapp_seq += 1
            else:
                folder = self._current_hla or "misc"
                if folder not in self._run:
                    self._run[folder] = 0
                    self._retry[folder] = 0
                    self._group_used[folder] = set()
                # A repeated name signals a re-detection -> open the next retry.
                if name in self._group_used[folder]:
                    self._retry[folder] += 1
                    self._group_used[folder] = set()
                run = self._run[folder]
                retry = self._retry[folder]
                self._group_used[folder].add(name)
                # First capture of a run uses a double underscore (0__rgb); reruns add
                # the retry index (0_1_rgb). The extra '_' sorts before the digit in the
                # file explorer, so the base capture lists ahead of its reruns while all
                # of a run's files cluster.
                stem = f"{run}__{name}" if retry == 0 else f"{run}_{retry}_{name}"
        return seq, folder, run, retry, stem

    # -- public API ---------------------------------------------------------

    def log_user_input(self, source: str, payload: Any) -> None:
        """Record a single user input (e.g. a message from the web interface)."""
        if not self.enabled:
            return
        try:
            record = {**self._timestamp(), "source": source, "payload": payload}
            with self._lock:
                self._append_jsonl(self._user_inputs_path, record)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to log user input from {source}: {e}")

    def log_event(self, category: str, **fields: Any) -> None:
        """Record a structured event (task command, preference bundle, etc.)."""
        if not self.enabled:
            return
        try:
            record = {**self._timestamp(), "category": category, **fields}
            with self._lock:
                self._append_jsonl(self._events_path, record)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to log event '{category}': {e}")

    def begin_hla(self, hla_name: str) -> None:
        """Mark ``hla_name`` as the active skill for subsequent ``log_image`` calls.

        Images logged after this land in ``images/<hla_name>/``. Each call bumps the
        full-run index (0, 1, 2, ...) and resets the retry counter, so re-running a
        skill (e.g. after a teleop takeover) writes a new ``<run>_*`` set instead of
        overwriting the previous one. No-op (but safe to call) when disabled.
        """
        if not self.enabled:
            return
        try:
            with self._lock:
                self._run[hla_name] = self._run.get(hla_name, -1) + 1
                self._retry[hla_name] = 0
                self._group_used[hla_name] = set()
                self._current_hla = hla_name
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to begin HLA '{hla_name}': {e}")

    def log_image(
        self,
        name: str,
        image: Any,
        is_rgb: bool = False,
        ext: str = "png",
        **metadata: Any,
    ) -> str | None:
        """Save an image into the active skill's folder and index it.

        ``name`` is the semantic image label (``"rgb"``, ``"depth"``,
        ``"detection_mask"``, ...). The image is written as
        ``images/<hla>/<run>[_<retry>]_<name>.<ext>`` for whichever skill
        ``begin_hla`` last selected (falling back to ``images/misc/`` if none is
        active yet). Distinct names share the current prefix (``0_rgb``, ``0_depth``);
        re-logging a name already seen opens the next retry within the run
        (``0_1_rgb``, ``0_1_depth``), while a fresh ``begin_hla`` bumps the run
        (``1_rgb``). The special name ``"webapp"`` routes to ``images/webapp_images/`` keyed by a
        monotonic counter (``<n>_webapp.<ext>``) -- the faithful, ordered record of
        what was shown on the iPad, independent of any skill.

        ``image`` is an OpenCV-style ndarray. Pass ``is_rgb=True`` for frames in
        RGB order (e.g. head-perception camera frames) so they are converted to
        BGR before writing; web-interface images are already BGR. Extra keyword
        arguments are stored as metadata in ``images_index.jsonl``. Returns the
        saved path (str) or None on failure / when disabled.
        """
        if not self.enabled:
            return None
        if not _CV2_IMPORTED:
            print("[data_logger] cv2 unavailable; cannot log image.")
            return None
        if image is None:
            return None
        try:
            ts = self._timestamp()
            seq, folder, run, retry, stem = self._reserve_slot(name)

            category_dir = self.images_dir / folder
            category_dir.mkdir(parents=True, exist_ok=True)
            out_path = category_dir / f"{stem}.{ext}"

            to_write = image
            if is_rgb and _NUMPY_IMPORTED and getattr(image, "ndim", 0) == 3:
                to_write = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(out_path), to_write):
                print(f"[data_logger] cv2.imwrite returned False for {out_path}")
                return None

            record = {
                **ts,
                "seq": seq,
                "folder": folder,
                "name": name,
                "run": run,
                "retry": retry,
                "path": str(out_path.relative_to(self.day_dir)),
                **metadata,
            }
            with self._lock:
                self._append_jsonl(self._images_index_path, record)
            return str(out_path)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to log image '{name}': {e}")
            return None

    def log_json(self, name: str, payload: Any, **metadata: Any) -> str | None:
        """Write a structured JSON sidecar into the active skill's folder and index it.

        Uses the same run/retry grouping as ``log_image``, so ``detection_inputs`` from a
        capture lands as ``images/<hla>/<run>[_<retry>]_detection_inputs.json`` right
        beside that capture's ``..._rgb.png`` / ``..._depth.png``. The index row carries
        ``kind="json"`` so offline tooling can distinguish sidecars from images and join
        them to the frames by (folder, run, retry). Best-effort; returns the saved path or
        None on failure / when disabled.
        """
        if not self.enabled:
            return None
        if payload is None:
            return None
        try:
            ts = self._timestamp()
            seq, folder, run, retry, stem = self._reserve_slot(name)

            category_dir = self.images_dir / folder
            category_dir.mkdir(parents=True, exist_ok=True)
            out_path = category_dir / f"{stem}.json"
            out_path.write_text(
                json.dumps(payload, indent=2, default=str), encoding="utf-8"
            )

            record = {
                **ts,
                "seq": seq,
                "folder": folder,
                "name": name,
                "run": run,
                "retry": retry,
                "kind": "json",
                "path": str(out_path.relative_to(self.day_dir)),
                **metadata,
            }
            with self._lock:
                self._append_jsonl(self._images_index_path, record)
            return str(out_path)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to log json '{name}': {e}")
            return None

    def close(self) -> None:
        """Finalize metadata (end time + counts). Safe to call multiple times."""
        if not self.enabled:
            return
        try:
            with self._lock:
                self._write_metadata(closed=True)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to finalize metadata: {e}")
