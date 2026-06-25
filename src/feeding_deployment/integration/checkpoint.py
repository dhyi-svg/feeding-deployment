"""Checkpoint file store for resumable meal execution.

Owns the ``saved_states`` directory: the running skill counter, the numbered
``NN_<skill>.p`` checkpoints for the deterministic plate journey (prep + finish),
the four fixed ``after_*_pickup.p`` feeding recovery points, the always-current
``last_state.p``, and fresh-run clearing.

This is pure file/naming logic -- it has no robot, simulator, or preference
knowledge. Callers build the payload dict (world state, atoms, preference
snapshot) and hand it to :meth:`CheckpointStore.save`; the store stamps it with
the skill index / completed-skill / phase metadata and writes it to every
applicable filename. Keeping it dependency-free makes save/load testable without
the robot stack.
"""

from __future__ import annotations

import pickle
import re
from pathlib import Path
from typing import Any, Dict, List

# Feeding-phase pickup skills -> fixed recovery checkpoint names. Resuming from
# one of these lands the robot on the task-selection page with the tool/bite in
# the gripper, so the restored atoms match the real world. Keyed by the skill's
# behavior-tree filename (sans .yaml).
FEEDING_PICKUP_CHECKPOINTS = {
    "pick_utensil": "after_utensil_pickup",
    "pick_drink": "after_drink_pickup",
    "pick_wipe": "after_wipe_pickup",
    "acquire_bite": "after_bite_pickup",
}

LAST_STATE = "last_state"

# Standalone preference-session snapshot, overwritten the instant each correction
# is locked (decoupled from the per-skill sim checkpoints) so a resume restores the
# latest corrections regardless of which skill boundary / phase the crash fell on.
PREF_SNAPSHOT = "pref_session"

# Numbered per-meal checkpoints, e.g. "03_pick_plate_from_fridge.p".
_NUMBERED_RE = re.compile(r"^\d+_.*\.p$")


class CheckpointStore:
    """Manages checkpoint files under a single directory."""

    def __init__(self, directory: Path) -> None:
        self.dir = Path(directory)
        self.dir.mkdir(exist_ok=True)
        # Running counter over the deterministic (prep/finish) skills, so they
        # are named 01_, 02_, ... A resume seeds it from the loaded checkpoint.
        self.index = 0

    def path(self, name: str) -> Path:
        """Absolute path of a checkpoint by base name. A trailing '.p' is
        tolerated, so `--resume_from_state 01_meal_start` and `01_meal_start.p`
        both resolve to the same file."""
        if name.endswith(".p"):
            name = name[:-2]
        return self.dir / f"{name}.p"

    def clear_ephemeral(self) -> None:
        """Remove the previous meal's numbered ``NN_<skill>.p`` checkpoints.

        Their numbering only makes sense within a single meal's plan (e.g. a run
        that skips the microwave produces a different skill sequence), so a stale
        numbered file from another meal is a resume footgun. ``last_state.p``,
        ``after_*_pickup.p``, and manual ``*.pkl`` files are preserved."""
        if not self.dir.exists():
            return
        for f in self.dir.glob("*.p"):
            if _NUMBERED_RE.match(f.name):
                f.unlink()

    def _targets_for(self, phase: str, completed_skill: str) -> List[str]:
        """Base names (besides ``last_state``) to write for the just-completed
        skill. Advances the counter for prep/finish skills."""
        if phase in ("prep", "finish"):
            self.index += 1
            return [f"{self.index:02d}_{completed_skill}"]
        if phase == "feeding":
            named = FEEDING_PICKUP_CHECKPOINTS.get(completed_skill)
            if named is not None:
                return [named]
        return []

    def save(
        self,
        core_payload: Dict[str, Any],
        *,
        phase: str,
        completed_skill: str,
    ) -> List[Path]:
        """Stamp ``core_payload`` with checkpoint metadata and write it to
        ``last_state.p`` plus any numbered/named target for this skill. Returns
        the files written (last_state first)."""
        targets = self._targets_for(phase, completed_skill)
        payload = {
            **core_payload,
            "skill_index": self.index,
            "completed_skill": completed_skill,
            "phase": phase,
        }
        written = [self.path(LAST_STATE), *[self.path(n) for n in targets]]
        for target in written:
            with open(target, "wb") as f:
                pickle.dump(payload, f)
        return written

    def load(self, name: str) -> Dict[str, Any]:
        """Read a checkpoint by base name and resume the counter from it."""
        target = self.path(name)
        if not target.exists():
            available = sorted(f.stem for f in self.dir.glob("*.p"))
            raise FileNotFoundError(
                f"No checkpoint '{target.name}' in {self.dir}. "
                f"Available checkpoints: {available or '(none)'}. "
                f"Pass the base name without '.p' (e.g. --resume_from_state last_state)."
            )
        with open(target, "rb") as f:
            payload = pickle.load(f)
        self.index = payload["skill_index"]
        return payload

    def save_pref(self, state: Dict[str, Any]) -> None:
        """Overwrite the standalone preference snapshot. Called on every locked
        correction so the latest preference state is always durable, independent
        of the sim checkpoints."""
        with open(self.path(PREF_SNAPSHOT), "wb") as f:
            pickle.dump(state, f)

    def load_pref(self) -> Dict[str, Any] | None:
        """Return the latest standalone preference snapshot, or None if absent or
        unreadable."""
        path = self.path(PREF_SNAPSHOT)
        if not path.exists():
            return None
        try:
            with open(path, "rb") as f:
                return pickle.load(f)
        except Exception as e:  # noqa: BLE001 - a corrupt snapshot must not block resume
            print(f"[checkpoint] Failed to read {path.name}: {e}")
            return None

    def clear_pref(self) -> None:
        """Remove the standalone preference snapshot (fresh-run reset)."""
        self.path(PREF_SNAPSHOT).unlink(missing_ok=True)
