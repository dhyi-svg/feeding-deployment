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
import shutil
import threading
import time
from contextlib import contextmanager
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

try:
    import rospy
    from std_msgs.msg import String as _RosString

    _ROSPY_IMPORTED = True
except ModuleNotFoundError:
    _ROSPY_IMPORTED = False


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

        # /deployment/annotations mirror: every log_event (plus meal start/end
        # metadata) is republished as a JSON String so the per-meal rosbags are
        # self-annotating -- a bag joined with nothing else still carries the
        # meal timeline. Publishes regardless of `enabled` (the bag recorder has
        # its own on/off), best-effort, and never blocks or raises.
        self._ann_pub = None
        self._ann_attempts = 0
        if _ROSPY_IMPORTED:
            try:
                self._ann_pub = rospy.Publisher(
                    "/deployment/annotations", _RosString, queue_size=50)
                # A message published immediately after Publisher() is dropped
                # before subscribers connect; give an already-running recorder
                # up to 2 s to attach so the meal_start annotation lands.
                deadline = time.time() + 2.0
                while (self._ann_pub.get_num_connections() == 0
                       and time.time() < deadline):
                    time.sleep(0.05)
            except Exception:  # noqa: BLE001 - e.g. node not initialized (tests)
                self._ann_pub = None

        if not self.enabled:
            self.day_dir = None
            print("[data_logger] No day provided; per-day release logging is DISABLED.")
            self._annotate("meal_start", user=self.user, day=None, release_logging=False)
            return

        self.day_dir = self.state_dir / f"day_{day:02d}"
        self.images_dir = self.day_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)

        self._user_inputs_path = self.day_dir / "user_inputs.jsonl"
        self._events_path = self.day_dir / "events.jsonl"
        self._images_index_path = self.day_dir / "images_index.jsonl"
        self._metadata_path = self.day_dir / "metadata.json"
        # Held-out pre-meal latent self-report (see integration/pre_meal_survey.py).
        # Its own per-day file, isolated from events.jsonl and from the
        # preference-learning memory tree, so it can never leak into prediction.
        self._pre_meal_path = self.day_dir / "pre_meal.jsonl"

        # Relaunch into an existing day (mid-run restart): resume the monotonic
        # counters from what prior sessions wrote so we don't reset to 0 and
        # overwrite earlier files / under-count images.
        self._resume_counters_from_disk()

        self._write_metadata(closed=False)
        print(f"[data_logger] Logging day {day:02d} for user '{self.user}' to {self.day_dir}")
        self._annotate("meal_start", user=self.user, day=self.day,
                       release_logging=True, day_dir=str(self.day_dir))

    # -- internal helpers ---------------------------------------------------

    @staticmethod
    def _timestamp() -> dict[str, Any]:
        now = time.time()
        return {"epoch": now, "iso": datetime.fromtimestamp(now).isoformat()}

    def _annotate(self, category: str, **fields: Any) -> None:
        """Mirror an event onto /deployment/annotations (JSON String).

        Best-effort by design: any failure is swallowed, and if the publisher
        could not be created eagerly (node not yet initialized), creation is
        retried lazily a bounded number of times, then given up on.
        """
        try:
            if not _ROSPY_IMPORTED:
                return
            if self._ann_pub is None:
                if self._ann_attempts >= 20:
                    return
                self._ann_attempts += 1
                try:
                    self._ann_pub = rospy.Publisher(
                        "/deployment/annotations", _RosString, queue_size=50)
                except Exception:  # noqa: BLE001
                    return
            record = {**self._timestamp(), "category": category, **fields}
            self._ann_pub.publish(_RosString(data=json.dumps(record, default=str)))
        except Exception:  # noqa: BLE001 - annotations must never disturb a meal
            pass

    def _append_jsonl(self, path: Path, record: dict[str, Any]) -> None:
        # Caller must hold the lock.
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, default=str) + "\n")

    def _resume_counters_from_disk(self) -> None:
        """Rehydrate the monotonic image counters after a relaunch into an
        existing ``day_NN`` dir.

        The in-memory counters (``_image_seq``, ``_webapp_seq``, per-folder
        ``_run``) reset to 0 on every process start, so a mid-run restart would
        re-issue filenames the previous session already wrote and silently
        overwrite them, and would under-count ``images_logged`` in metadata
        (rajat_pilot 2026-07-16: three restarts overwrote 38 images incl. the
        first open_microwave detection set, and metadata showed 51/202 images).
        Resume each counter past what is already on disk -- the disk/index is
        the source of truth, mirroring ``snapshot_state_file``'s next-free-index
        scan. Best-effort: never let resume IO break logging.

        ``_retry`` / ``_group_used`` are intentionally NOT resumed: they are
        per-run scratch state, and each resumed folder gets a fresh run index
        from ``begin_hla`` (get(folder,-1)+1 -> max_on_disk+1), so a new run's
        files never collide with the prior session's.
        """
        # Global capture seq: continue past the highest seq in the index. max+1
        # (not row count) stays correct even though a prior restart already
        # wrote duplicate low seqs into the file.
        try:
            if self._images_index_path.exists():
                max_seq = -1
                for line in self._images_index_path.read_text(encoding="utf-8").splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        row = json.loads(line)
                    except Exception:  # noqa: BLE001 - skip a torn last line
                        continue
                    if isinstance(row.get("seq"), int):
                        max_seq = max(max_seq, row["seq"])
                self._image_seq = max_seq + 1
        except Exception:  # noqa: BLE001
            pass

        # Per-folder run index and the webapp counter: derive from the files
        # actually on disk (the true anti-collision target).
        try:
            if self.images_dir.exists():
                for folder_dir in self.images_dir.iterdir():
                    if not folder_dir.is_dir():
                        continue
                    max_lead = -1
                    for f in folder_dir.iterdir():
                        if not f.is_file():
                            continue
                        head = f.stem.split("_", 1)[0]
                        if head.isdigit():
                            max_lead = max(max_lead, int(head))
                    if max_lead < 0:
                        continue
                    if folder_dir.name == "webapp_images":
                        self._webapp_seq = max_lead + 1
                    else:
                        # begin_hla will bump this to max_lead+1 for the new run.
                        self._run[folder_dir.name] = max_lead
                        self._retry[folder_dir.name] = 0
                        self._group_used[folder_dir.name] = set()
        except Exception:  # noqa: BLE001
            pass

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
        # Merge with any existing metadata, preserving the ORIGINAL start time
        # across relaunches into the same day. existing.update(meta) alone let
        # the new (later) `started` clobber the true one on every restart
        # (rajat_pilot 2026-07-16: logged 22:03:54 for a run that began 21:33:51).
        try:
            if self._metadata_path.exists():
                existing = json.loads(self._metadata_path.read_text(encoding="utf-8"))
                if not closed and "started" in existing:
                    meta["started"] = existing["started"]
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
        """Record a structured event (task command, preference bundle, etc.).

        Also mirrored onto /deployment/annotations (even when per-day release
        logging is disabled) so the meal rosbags carry the semantic timeline.
        """
        self._annotate(category, **fields)
        if not self.enabled:
            return
        try:
            record = {**self._timestamp(), "category": category, **fields}
            with self._lock:
                self._append_jsonl(self._events_path, record)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to log event '{category}': {e}")

    def log_pre_meal(self, **fields: Any) -> None:
        """Append a held-out pre-meal latent self-report record to pre_meal.jsonl.

        Deliberately NOT mirrored to /deployment/annotations and kept in its own
        per-day file (separate from events.jsonl and the preference-learning
        memory): these self-reports are validation ground truth and must never be
        fed back into prediction. Best-effort; never raises.
        """
        if not self.enabled:
            return
        try:
            record = {**self._timestamp(), **fields}
            with self._lock:
                self._append_jsonl(self._pre_meal_path, record)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to log pre-meal record: {e}")

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

    @contextmanager
    def skill_execution(self, hla_name: str, **fields: Any):
        """Wrap one skill-execution attempt in a ``skill_execute`` outcome event.

        Purely observational: every exception is re-raised unchanged
        (BaseException included, so SIGINT / sys.exit propagate exactly as
        without the wrapper), and the event is emitted from ``finally`` via
        best-effort ``log_event``, so logging can never alter the caller's
        control flow. Outcomes: ``success``; ``takeover`` (a
        TeleopTakeoverException, classified by class name to avoid an import
        cycle, with its ``redo_current`` flag); ``aborted``
        (SystemExit/KeyboardInterrupt); otherwise ``failed`` with the raised
        exception, verbatim plus its innermost frame, as ``failure_reason`` --
        the failure taxonomy is clustered offline from these strings, not
        chosen at log time. ``run`` joins the event to ``images/<hla>/<run>_*``
        from the same execution.
        """
        start = time.time()
        with self._lock:
            run_idx = self._run.get(hla_name)
        result: dict[str, Any] = {"outcome": "success"}
        try:
            yield
        except BaseException as e:  # noqa: BLE001 - observe and re-raise, never swallow
            if type(e).__name__ == "TeleopTakeoverException":
                result = {"outcome": "takeover",
                          "redo": bool(getattr(e, "redo_current", False))}
            elif isinstance(e, (KeyboardInterrupt, SystemExit)):
                result = {"outcome": "aborted",
                          "failure_reason": type(e).__name__}
            else:
                where = ""
                tb = e.__traceback__
                while tb is not None and tb.tb_next is not None:
                    tb = tb.tb_next
                if tb is not None:
                    code = tb.tb_frame.f_code
                    where = (f" @ {Path(code.co_filename).name}:"
                             f"{tb.tb_lineno} in {code.co_name}")
                result = {"outcome": "failed",
                          "failure_reason": f"{type(e).__name__}: {e}{where}"}
            raise
        finally:
            self.log_event("skill_execute", hla=hla_name, run=run_idx,
                           start_epoch=start,
                           duration_s=round(time.time() - start, 3),
                           **result, **fields)

    def snapshot_state_file(self, path: Any) -> None:
        """Archive a versioned copy of a mutable user-level state file.

        Calibration/pose pickles under ``state_dir`` are rewritten in place on
        every re-perception, so only their last state would survive the day.
        Called right after such a write, this copies the file to
        ``day_<NN>/<stem>_log/<stem>_<N>.<ext>`` (N = 0, 1, 2, ... within the
        day, mirroring the ``food_detection_log`` idiom) and logs a
        ``state_snapshot`` event, so every calibration change is timestamped in
        events.jsonl and on /deployment/annotations.
        """
        if not self.enabled:
            return
        try:
            src = Path(path)
            if not src.exists():
                return
            dest_dir = self.day_dir / f"{src.stem}_log"
            dest_dir.mkdir(parents=True, exist_ok=True)
            with self._lock:
                n = 0
                while (dest_dir / f"{src.stem}_{n}{src.suffix}").exists():
                    n += 1
                dest = dest_dir / f"{src.stem}_{n}{src.suffix}"
                shutil.copy2(src, dest)
            self.log_event("state_snapshot", name=src.stem,
                           path=str(dest.relative_to(self.day_dir)))
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to snapshot state file '{path}': {e}")

    def _snapshot_actuated_state(self) -> None:
        """Copy end-of-run actuated state into the day dir.

        ``behavior_trees/`` (the parameters skills actually ran with) and
        ``flair_history.txt`` live at the user level and are mutated in place
        across days; copying them at close preserves each day's final state.
        Overwrites earlier copies from the same day, so a day with several
        run.py sessions keeps its last state.
        """
        try:
            bt_src = self.state_dir / "behavior_trees"
            if bt_src.is_dir():
                shutil.copytree(bt_src, self.day_dir / "behavior_trees",
                                dirs_exist_ok=True)
            flair_src = self.state_dir / "flair_history.txt"
            if flair_src.exists():
                shutil.copy2(flair_src, self.day_dir / "flair_history.txt")
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to snapshot actuated state: {e}")

    def close(self) -> None:
        """Finalize metadata (end time + counts). Safe to call multiple times."""
        self._annotate("meal_end", user=self.user, day=self.day,
                       images_logged=self._image_seq)
        if not self.enabled:
            return
        self._snapshot_actuated_state()
        try:
            with self._lock:
                self._write_metadata(closed=True)
        except Exception as e:  # noqa: BLE001
            print(f"[data_logger] Failed to finalize metadata: {e}")
