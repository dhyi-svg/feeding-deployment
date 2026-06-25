"""Tests for the CheckpointStore (resumable meal execution file store).

Run with:
    PYTHONPATH=src python -m pytest tests/test_checkpoint.py -v
or, since CheckpointStore has no heavy dependencies, directly:
    PYTHONPATH=src python tests/test_checkpoint.py

These exercise the pure save/load + naming + clearing logic without the robot,
simulator, or preference stack. The payload contents are stand-ins (None world
state, plain sets/dicts) -- the store treats the core payload opaquely.
"""

from __future__ import annotations

import pickle
import tempfile
from pathlib import Path

from feeding_deployment.integration.checkpoint import (
    CheckpointStore,
    FEEDING_PICKUP_CHECKPOINTS,
    LAST_STATE,
    PREF_SNAPSHOT,
)


def _core(atoms=None):
    """A representative opaque core payload."""
    return {
        "sim_state": None,
        "atoms": atoms if atoms is not None else {"GripperFree()", "PlateAt(fridge)"},
        "preference_session": {"bundle": {"robot_speed": "fast"}, "finalized": {"robot_speed"}},
    }


def _names(store: CheckpointStore):
    return sorted(p.name for p in store.dir.glob("*.p"))


def test_prep_skill_numbers_and_increments(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    assert s.index == 0
    written = s.save(_core(), phase="prep", completed_skill="open_fridge")
    assert s.index == 1
    assert {p.name for p in written} == {"last_state.p", "01_open_fridge.p"}

    s.save(_core(), phase="prep", completed_skill="pick_plate_from_fridge")
    assert s.index == 2
    assert "02_pick_plate_from_fridge.p" in _names(s)


def test_finish_phase_continues_numbering(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    s.save(_core(), phase="prep", completed_skill="place_plate_on_table")  # 01
    written = s.save(_core(), phase="finish", completed_skill="place_plate_in_sink")
    assert s.index == 2
    assert {p.name for p in written} == {"last_state.p", "02_place_plate_in_sink.p"}


def test_feeding_pickups_get_named_recovery_state_no_number(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    for skill, expected in FEEDING_PICKUP_CHECKPOINTS.items():
        s.index = 0  # numbering must never advance in the feeding phase
        sub = CheckpointStore(s.dir)
        written = sub.save(_core(), phase="feeding", completed_skill=skill)
        assert sub.index == 0
        assert {p.name for p in written} == {"last_state.p", f"{expected}.p"}


def test_feeding_non_pickup_writes_only_last_state(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    written = s.save(_core(), phase="feeding", completed_skill="transfer_utensil")
    assert s.index == 0
    assert {p.name for p in written} == {"last_state.p"}


def test_save_load_roundtrip_and_metadata(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    atoms = {"Holding(utensil)"}
    s.save(_core(atoms), phase="feeding", completed_skill="acquire_bite")

    payload = s.load("after_bite_pickup")
    assert payload["atoms"] == atoms
    assert payload["completed_skill"] == "acquire_bite"
    assert payload["phase"] == "feeding"
    assert payload["skill_index"] == 0
    assert payload["preference_session"]["finalized"] == {"robot_speed"}
    # last_state holds the same payload.
    assert s.load(LAST_STATE)["completed_skill"] == "acquire_bite"


def test_load_resumes_the_counter(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    s.save(_core(), phase="prep", completed_skill="open_fridge")            # 01
    s.save(_core(), phase="prep", completed_skill="pick_plate_from_fridge")  # 02
    s.save(_core(), phase="prep", completed_skill="close_fridge")            # 03

    # Fresh store (new run) resumes from the 02 checkpoint and continues at 03.
    s2 = CheckpointStore(tmp_path)
    s2.load("02_pick_plate_from_fridge")
    assert s2.index == 2
    written = s2.save(_core(), phase="prep", completed_skill="close_fridge")
    assert "03_close_fridge.p" in {p.name for p in written}


def test_clear_ephemeral_preserves_named_and_manual_files(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    s.save(_core(), phase="prep", completed_skill="open_fridge")             # 01_*
    s.save(_core(), phase="finish", completed_skill="place_plate_in_sink")   # 02_*
    s.save(_core(), phase="feeding", completed_skill="pick_drink")           # after_drink_pickup
    # A manual artifact unrelated to auto-checkpointing.
    (tmp_path / "study_drink_pickup_pos.pkl").write_bytes(pickle.dumps({"x": 1}))

    s.clear_ephemeral()

    remaining = set(p.name for p in tmp_path.iterdir())
    assert "01_open_fridge.p" not in remaining
    assert "02_place_plate_in_sink.p" not in remaining
    assert "last_state.p" in remaining
    assert "after_drink_pickup.p" in remaining
    assert "study_drink_pickup_pos.pkl" in remaining


def test_pref_snapshot_roundtrip_and_clear(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    # Absent snapshot -> None.
    assert s.load_pref() is None

    snap = {"context": {"meal": "x"}, "bundle": {"transfer_mode": "fork"},
            "finalized": {"transfer_mode"}, "corrected": {"transfer_mode": "fork"}}
    s.save_pref(snap)
    assert (tmp_path / f"{PREF_SNAPSHOT}.p").exists()
    assert s.load_pref() == snap

    # Overwrites in place (latest correction wins).
    snap2 = {**snap, "corrected": {"transfer_mode": "spoon"}}
    s.save_pref(snap2)
    assert s.load_pref()["corrected"] == {"transfer_mode": "spoon"}

    s.clear_pref()
    assert s.load_pref() is None
    s.clear_pref()  # idempotent / missing_ok


def test_clear_ephemeral_does_not_touch_pref_snapshot(tmp_path: Path):
    s = CheckpointStore(tmp_path)
    s.save(_core(), phase="prep", completed_skill="open_fridge")  # 01_*
    s.save_pref({"bundle": {}, "finalized": set(), "corrected": {}, "context": {}})
    # The standalone snapshot survives the numbered-checkpoint sweep; it is only
    # dropped by the explicit clear_pref() on a fresh run.
    s.clear_ephemeral()
    assert s.load_pref() is not None


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        with tempfile.TemporaryDirectory() as d:
            fn(Path(d))
        print(f"PASS {fn.__name__}")
    print(f"\nAll {len(fns)} CheckpointStore tests passed.")


if __name__ == "__main__":
    _run_all()
